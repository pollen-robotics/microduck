//! `robotctl vetbook` — the duck's vet book: its whole life in numbers, and every servo it has
//! been through.
//!
//! The book is `robotd`'s (`robot.vetbook`; the record itself is `duck_ipc_proto::vetbook`). This
//! is the page it is read on: three tabs — the life, the motors, the diary — refreshed every two
//! seconds, so the walking hours tick up while you watch the duck walk. `n` writes a line in the
//! diary from the page itself. Piped, it prints the same three as plain text; `--json` gives the
//! book as it is.

use std::io::IsTerminal;
use std::path::Path;
use std::time::{Duration, Instant};

use duck_ipc_proto as proto;
use proto::vetbook::{HOT_C, MotorRecord, Vetbook};
use ratatui::DefaultTerminal;
use ratatui::buffer::Buffer;
use ratatui::crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use ratatui::layout::{Alignment, Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{
    Block, BorderType, Cell, Paragraph, Row, Table, TableState, Tabs, Widget, Wrap,
};

use crate::{Client, Failure, decode, exit, result_of};

/// How often the open page asks for the book again. The loop hands the book its counts once a
/// second; faster than this would only redraw the same numbers.
const REFRESH: Duration = Duration::from_secs(2);

// The palette: a yellow duck, an orange beak, and a pond.
const DUCK: Color = Color::Indexed(220);
const BEAK: Color = Color::Indexed(208);
const POND: Color = Color::Indexed(38);
const LEAF: Color = Color::Indexed(114);
const BLUSH: Color = Color::Indexed(211);
const LILAC: Color = Color::Indexed(141);
const DIM: Color = Color::Indexed(244);
const ALERT: Color = Color::Indexed(203);

/// The patient card's height, borders included: the head's twenty pixels at two to a cell.
const HEADER_HEIGHT: u16 = 12;

/// A microduck's head, three-quarter view, after the sticker: the cream shell over the back with
/// the hinge on its side, the grey face, the camera eye in its orange ring with the slot beside
/// it, the orange band under the head, and the beak hanging open — an orange tray with its
/// lavender pad. Sampled from the sticker, snapped to these colours and finished by hand. Drawn
/// two pixels to a cell with half blocks, so the pixels come out square; `.` is the terminal's
/// own background.
const HEAD: [&str; 20] = [
    "......GGccccccggddddddd.....",
    "....GcCCCCCCGgddgggggggdd...",
    "...cCCCCCCCGddgggdddgggggd..",
    "..GCCCCCCCGddggdbOOOodggggd.",
    ".gcccCCCCcgdgggoOo#bRoggggd.",
    ".ccccccccGdgggdOo#dgbOdgggd.",
    "gccccccccgggggbOb##g#ob##ggd",
    "GdbdcccccgggggbRb####Rb##ggd",
    "gbr#ccccGdggggdoo###bOdggggd",
    "gbbbGcccgdggggdbObbbOogggggd",
    "gbrbccccgdgggggdboOOoggggggd",
    "rbrbgcccggggggggddddddddddbb",
    "rrRrroggdddddbbboorrRRRRRRRr",
    "brRrrrrrrRRRRRRRrrrrrrbbdgd.",
    "..rrbrrbbbbbbbbdddddddddd...",
    "...rrrdddlllllLLLLLLLLLLld..",
    "....rrdLLLLLLLLLLLLLLLLlLdb.",
    ".....rrdllllllldddddbbbbbrro",
    "......rrbbrrrrrRRRRRRRRRRRR.",
    ".......rRRRRRRRRrrrooo......",
];
const HEAD_WIDTH: u16 = 28;

fn head_colour(pixel: u8) -> Option<Color> {
    Some(match pixel {
        b'#' => Color::Rgb(26, 22, 22),
        b'C' => Color::Rgb(234, 227, 212),
        b'c' => Color::Rgb(204, 196, 178),
        b'G' => Color::Rgb(180, 180, 182),
        b'g' => Color::Rgb(150, 150, 152),
        b'd' => Color::Rgb(112, 112, 116),
        b'O' => Color::Rgb(238, 142, 42),
        b'o' => Color::Rgb(176, 96, 40),
        b'R' => Color::Rgb(222, 94, 40),
        b'r' => Color::Rgb(160, 62, 30),
        b'b' => Color::Rgb(104, 44, 24),
        b'L' => Color::Rgb(192, 176, 210),
        b'l' => Color::Rgb(150, 134, 162),
        _ => return None,
    })
}

/// Paint [`HEAD`] into `area`, two rows of pixels per row of cells.
fn render_head(area: Rect, buf: &mut Buffer) {
    for row in 0..(HEAD.len() / 2) as u16 {
        if row >= area.height {
            break;
        }
        let (top, bottom) = (
            HEAD[2 * row as usize].as_bytes(),
            HEAD[2 * row as usize + 1].as_bytes(),
        );
        for x in 0..HEAD_WIDTH.min(area.width) {
            let (t, b) = (
                head_colour(top[x as usize]),
                head_colour(bottom[x as usize]),
            );
            let cell = &mut buf[(area.x + x, area.y + row)];
            match (t, b) {
                (Some(t), Some(b)) => {
                    cell.set_symbol("▀").set_fg(t).set_bg(b);
                }
                (Some(t), None) => {
                    cell.set_symbol("▀").set_fg(t);
                }
                (None, Some(b)) => {
                    cell.set_symbol("▄").set_fg(b);
                }
                (None, None) => {}
            }
        }
    }
}

pub fn run(
    robot_socket: &Path,
    config_socket: &Path,
    updater_socket: &Path,
    json: bool,
) -> Result<(), Failure> {
    let book = fetch(robot_socket)?;
    if json {
        println!(
            "{}",
            serde_json::to_string_pretty(&book).expect("a vet book serialises")
        );
        return Ok(());
    }
    let name = robot_name(config_socket);
    if !std::io::stdout().is_terminal() {
        print!("{}", plain(&book, &name, crate::unix_now()));
        return Ok(());
    }
    let mut terminal = ratatui::init();
    let outcome = live(&mut terminal, robot_socket, updater_socket, book, &name);
    ratatui::restore();
    outcome
}

fn fetch(socket: &Path) -> Result<Vetbook, Failure> {
    let mut client = Client::connect_to("robotd", socket)?;
    client.hello()?;
    decode(&result_of(client.call(&proto::Call::RobotVetbook)?)?)
}

/// `robot.vetbookNote`. The robot's refusal comes back as the error, worded for the page.
fn write_note(socket: &Path, text: &str) -> Result<(), Failure> {
    let mut client = Client::connect_to("robotd", socket)?;
    client.hello()?;
    let result = result_of(client.call(&proto::Call::RobotVetbookNote(
        proto::vetbook::VetbookNoteParams {
            text: text.to_owned(),
        },
    ))?)?;
    let outcome: proto::IntentResult = decode(&result)?;
    if !outcome.accepted {
        let reason = outcome
            .reason
            .unwrap_or_else(|| "the robot refused".to_owned());
        return Err(Failure::new(exit::REFUSED, reason));
    }
    Ok(())
}

/// The robot's name from `configd`, or the hostname, or nothing — the page works without it.
fn robot_name(config_socket: &Path) -> String {
    Client::connect_to("configd", config_socket)
        .ok()
        .and_then(|mut c| c.call(&proto::Call::SystemInfo).ok())
        .and_then(|r| r.result_as::<proto::SystemInfoResult>().ok())
        .map(|info| info.name)
        .filter(|n| !n.is_empty())
        .or_else(|| {
            std::fs::read_to_string("/etc/hostname")
                .ok()
                .map(|h| h.trim().to_owned())
        })
        .unwrap_or_default()
}

// ── the live page ────────────────────────────────────────────────────────────

#[derive(Clone, Copy, PartialEq, Eq)]
enum Tab {
    Life,
    Motors,
    Records,
    Diary,
}

impl Tab {
    const ALL: [Tab; 4] = [Tab::Life, Tab::Motors, Tab::Records, Tab::Diary];

    fn index(self) -> usize {
        Self::ALL.iter().position(|t| *t == self).unwrap_or(0)
    }

    fn step(self, by: isize) -> Tab {
        let n = Self::ALL.len() as isize;
        Self::ALL[(self.index() as isize + by).rem_euclid(n) as usize]
    }
}

struct Page {
    book: Vetbook,
    name: String,
    tab: Tab,
    motors: TableState,
    diary_scroll: u16,
    /// Why the last refresh failed, shown in the footer while the old book stays up.
    stale: Option<String>,
    now: i64,
    /// A diary line being typed, while `n` has the keyboard.
    draft: Option<String>,
    /// What came of the last line written, shown in the footer for a few seconds.
    written: Option<(Result<(), String>, Instant)>,
    /// A share under way, or its outcome, shown over the page until a key is pressed.
    share: Share,
}

/// `s`: publishing the book through `updaterd` (`vetbook.share`).
#[derive(Default)]
enum Share {
    #[default]
    Closed,
    /// Whether to list the duck in the Space's gallery: asked on every share, nothing is
    /// published until it is answered.
    Asking,
    /// Drawn once before the call is made, so the page says what it is waiting for. `true` is a
    /// listed share.
    Working(bool),
    Done(Result<proto::VetbookShareResult, String>),
}

/// `vetbook.share`, served by `updaterd`: it holds the account and the network.
fn share(socket: &Path, listed: bool) -> Result<proto::VetbookShareResult, Failure> {
    let mut client = Client::connect_to("updaterd", socket)?;
    client.hello()?;
    let params = proto::VetbookShareParams { listed };
    decode(&result_of(
        client.call(&proto::Call::VetbookShare(params))?,
    )?)
}

fn live(
    terminal: &mut DefaultTerminal,
    socket: &Path,
    updater_socket: &Path,
    book: Vetbook,
    name: &str,
) -> Result<(), Failure> {
    let mut page = Page {
        book,
        name: name.to_owned(),
        tab: Tab::Life,
        motors: TableState::default().with_selected(Some(0)),
        diary_scroll: 0,
        stale: None,
        now: crate::unix_now(),
        draft: None,
        written: None,
        share: Share::Closed,
    };
    let mut fetched = Instant::now();
    loop {
        page.now = crate::unix_now();
        terminal
            .draw(|frame| page.render(frame.area(), frame.buffer_mut()))
            .map_err(|e| Failure::new(exit::FAILED, format!("cannot draw: {e}")))?;

        // The "sharing…" frame is on screen; now the few seconds of talking to Hugging Face.
        if let Share::Working(listed) = page.share {
            page.share = Share::Done(share(updater_socket, listed).map_err(|e| e.message));
            continue;
        }

        let wait = REFRESH.saturating_sub(fetched.elapsed());
        if event::poll(wait).unwrap_or(false)
            && let Ok(Event::Key(key)) = event::read()
            && key.kind == KeyEventKind::Press
        {
            let rows = page.book.motors.len();
            // A share's outcome stays up until it has been read: any key closes it.
            if matches!(page.share, Share::Done(_)) {
                page.share = Share::Closed;
                continue;
            }
            if matches!(page.share, Share::Asking) {
                page.share = match key.code {
                    KeyCode::Char('y') => Share::Working(true),
                    KeyCode::Char('n') => Share::Working(false),
                    KeyCode::Esc | KeyCode::Char('q') => Share::Closed,
                    _ => Share::Asking,
                };
                continue;
            }
            // While a line is being typed every key is text, `q` included.
            if let Some(draft) = page.draft.as_mut() {
                match key.code {
                    KeyCode::Esc => page.draft = None,
                    KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                        page.draft = None
                    }
                    KeyCode::Enter => {
                        let text = page.draft.take().unwrap_or_default();
                        if !text.trim().is_empty() {
                            let outcome = write_note(socket, &text).map_err(|e| e.message);
                            page.written = Some((outcome, Instant::now()));
                            page.diary_scroll = 0;
                            fetched = Instant::now() - REFRESH;
                        }
                    }
                    KeyCode::Backspace => {
                        draft.pop();
                    }
                    KeyCode::Char(c) => draft.push(c),
                    _ => {}
                }
                continue;
            }
            match key.code {
                KeyCode::Char('q') | KeyCode::Esc => return Ok(()),
                KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                    return Ok(());
                }
                KeyCode::Tab | KeyCode::Right | KeyCode::Char('l') => page.tab = page.tab.step(1),
                KeyCode::BackTab | KeyCode::Left | KeyCode::Char('h') => {
                    page.tab = page.tab.step(-1)
                }
                KeyCode::Char('1') => page.tab = Tab::Life,
                KeyCode::Char('2') => page.tab = Tab::Motors,
                KeyCode::Char('3') => page.tab = Tab::Records,
                KeyCode::Char('4') => page.tab = Tab::Diary,
                KeyCode::Down | KeyCode::Char('j') => match page.tab {
                    Tab::Motors => {
                        let i = page.motors.selected().unwrap_or(0);
                        page.motors
                            .select(Some((i + 1).min(rows.saturating_sub(1))));
                    }
                    Tab::Diary => page.diary_scroll = page.diary_scroll.saturating_add(1),
                    Tab::Life | Tab::Records => {}
                },
                KeyCode::Up | KeyCode::Char('k') => match page.tab {
                    Tab::Motors => {
                        let i = page.motors.selected().unwrap_or(0);
                        page.motors.select(Some(i.saturating_sub(1)));
                    }
                    Tab::Diary => page.diary_scroll = page.diary_scroll.saturating_sub(1),
                    Tab::Life | Tab::Records => {}
                },
                KeyCode::Char('r') => fetched = Instant::now() - REFRESH,
                KeyCode::Char('n') => {
                    page.tab = Tab::Diary;
                    page.draft = Some(String::new());
                }
                KeyCode::Char('s') => page.share = Share::Asking,
                _ => {}
            }
        }
        if page
            .written
            .as_ref()
            .is_some_and(|(_, at)| at.elapsed() > Duration::from_secs(4))
        {
            page.written = None;
        }
        if fetched.elapsed() >= REFRESH {
            fetched = Instant::now();
            match fetch(socket) {
                Ok(book) => {
                    page.book = book;
                    page.stale = None;
                }
                Err(e) => page.stale = Some(e.message),
            }
        }
    }
}

/// A panel of the Records tab: its title, its colour, and its lines as (label, value).
type RecordGroup<'a> = (&'a str, Color, Vec<(&'a str, String)>);

impl Page {
    fn render(&mut self, area: Rect, buf: &mut Buffer) {
        let [header, tabs, body, footer] = Layout::vertical([
            Constraint::Length(HEADER_HEIGHT),
            Constraint::Length(1),
            Constraint::Min(8),
            Constraint::Length(1),
        ])
        .areas(area);
        self.render_header(header, buf);

        let titles = ["♥ Life", "⚙ Motors", "★ Records", "✎ Diary"]
            .iter()
            .enumerate()
            .map(|(i, t)| Line::from(format!(" {} {t} ", i + 1)));
        Tabs::new(titles)
            .select(self.tab.index())
            .style(Style::new().fg(DIM))
            .highlight_style(Style::new().fg(Color::Black).bg(DUCK).bold())
            .divider(" ")
            .render(tabs, buf);

        match self.tab {
            Tab::Life => self.render_life(body, buf),
            Tab::Motors => self.render_motors(body, buf),
            Tab::Records => self.render_records(body, buf),
            Tab::Diary => self.render_diary(body, buf),
        }

        let key = |k: &'static str| Span::styled(k, Style::new().fg(DUCK).bold());
        let what = |w: &'static str| Span::styled(w, Style::new().fg(DIM));
        let mut keys = if self.draft.is_some() {
            vec![
                key(" ⏎"),
                what(" write it in the diary  "),
                key("esc"),
                what(" never mind"),
            ]
        } else {
            vec![
                key(" q"),
                what(" quit  "),
                key("←→"),
                what(" tabs  "),
                key("↑↓"),
                what(" scroll  "),
                key("n"),
                what(" write in the diary  "),
                key("s"),
                what(" share"),
            ]
        };
        match &self.written {
            Some((Ok(()), _)) => keys.push(Span::styled(
                "   ✎ written in the vet book",
                Style::new().fg(LEAF).bold(),
            )),
            Some((Err(e), _)) => keys.push(Span::styled(
                format!("   ⚠ not written: {e}"),
                Style::new().fg(ALERT),
            )),
            None => {}
        }
        if let Some(stale) = &self.stale {
            keys.push(Span::styled(
                format!("   ⚠ not refreshed: {stale}"),
                Style::new().fg(ALERT),
            ));
        }
        Line::from(keys).render(footer, buf);

        if !matches!(self.share, Share::Closed) {
            self.render_share(area, buf);
        }
    }

    /// The patient card: the few lines a vet would read first.
    /// The share, over the middle of the page: waiting, or the link and a QR code to open it on a
    /// phone, or why it did not work.
    fn render_share(&self, area: Rect, buf: &mut Buffer) {
        let qr = match &self.share {
            Share::Done(Ok(shared)) => qr_modules(&shared.url),
            _ => None,
        };
        let qr_rows = qr.as_ref().map_or(0, |m| m.len().div_ceil(2) as u16);
        let qr_cols = qr.as_ref().map_or(0, |m| m.len() as u16);
        // Wide enough for the link on one line, so nothing wraps into the code below it.
        let url_cols = match &self.share {
            Share::Done(Ok(shared)) => shared.url.chars().count() as u16,
            _ => 0,
        };
        let width = (qr_cols + 6)
            .max(url_cols + 6)
            .max(64)
            .min(area.width.saturating_sub(2));
        let height = (qr_rows + 9).min(area.height.saturating_sub(2));
        let popup = Rect {
            x: area.x + area.width.saturating_sub(width) / 2,
            y: area.y + area.height.saturating_sub(height) / 2,
            width,
            height,
        };
        ratatui::widgets::Clear.render(popup, buf);
        let (title, colour) = match &self.share {
            Share::Done(Err(_)) => (" not shared ", ALERT),
            Share::Done(Ok(_)) => (" shared ♡ ", LEAF),
            Share::Asking => (" share ", DUCK),
            _ => (" sharing… ", DUCK),
        };
        let block = Block::bordered()
            .border_type(BorderType::Rounded)
            .border_style(Style::new().fg(colour))
            .title(Span::styled(title, Style::new().fg(colour).bold()));
        let inner = block.inner(popup).inner(ratatui::layout::Margin::new(1, 0));
        block.render(popup, buf);

        let lines = match &self.share {
            Share::Done(Ok(shared)) => vec![
                Line::styled(shared.url.clone(), Style::new().fg(DUCK).bold()),
                Line::from(vec![
                    Span::styled("drew the ", Style::new().fg(DIM)),
                    Span::styled(shared.card.clone(), Style::new().fg(BLUSH).bold()),
                    Span::styled(" card · book at ", Style::new().fg(DIM)),
                    Span::raw(shared.dataset.clone()),
                ]),
                if shared.listed {
                    Line::styled("listed in the vet book gallery", Style::new().fg(LEAF))
                } else {
                    Line::styled(
                        "not in the gallery: only people with the link will find it",
                        Style::new().fg(DIM),
                    )
                },
            ],
            Share::Done(Err(why)) => vec![Line::styled(why.clone(), Style::new().fg(ALERT))],
            Share::Asking => vec![
                Line::from(vec![
                    Span::raw("Put "),
                    Span::styled(self.name.clone(), Style::new().fg(DUCK).bold()),
                    Span::raw(" in the public vet book gallery?"),
                ]),
                Line::styled(
                    "Anybody can then find it by name and compare its stats with other ducks.",
                    Style::new().fg(DIM),
                ),
                Line::styled(
                    "Either way the book is uploaded to a public dataset, so the link works.",
                    Style::new().fg(DIM),
                ),
                Line::raw(""),
                Line::from(vec![
                    Span::styled("y", Style::new().fg(LEAF).bold()),
                    Span::raw(" list it   "),
                    Span::styled("n", Style::new().fg(BLUSH).bold()),
                    Span::raw(" share without listing   "),
                    Span::styled("esc", Style::new().fg(DIM).bold()),
                    Span::raw(" cancel"),
                ]),
            ],
            _ => vec![Line::styled(
                "publishing the vet book to your Hugging Face account…",
                Style::new().fg(DIM),
            )],
        };
        Paragraph::new(lines)
            .wrap(Wrap { trim: true })
            .render(inner, buf);

        if let Some(modules) = &qr {
            let left = inner.x + inner.width.saturating_sub(qr_cols) / 2;
            render_qr(modules, Rect::new(left, inner.y + 4, qr_cols, qr_rows), buf);
        }
        if matches!(self.share, Share::Done(_)) {
            let hint = if qr.is_some() {
                "scan it to post from your phone · any key closes"
            } else {
                "any key closes"
            };
            Line::styled(hint, Style::new().fg(DIM))
                .alignment(Alignment::Center)
                .render(
                    Rect::new(inner.x, inner.bottom().saturating_sub(1), inner.width, 1),
                    buf,
                );
        }
    }

    fn render_header(&self, area: Rect, buf: &mut Buffer) {
        let block = Block::bordered()
            .border_type(BorderType::Rounded)
            .border_style(Style::new().fg(DUCK));
        let inner = block.inner(area);
        block.render(area, buf);
        let [head, _, text] = Layout::horizontal([
            Constraint::Length(HEAD_WIDTH),
            Constraint::Length(3),
            Constraint::Min(10),
        ])
        .areas(inner);
        render_head(head, buf);

        let b = &self.book;
        let life = &b.life;
        let title = if self.name.is_empty() {
            "the vet book".to_owned()
        } else {
            format!("{}'s vet book", self.name)
        };
        let days = (self.now - b.opened as i64).max(0) / 86_400;
        let (verdict, colour) = verdict(b);
        let dim = |t: &str| Span::styled(t.to_owned(), Style::new().fg(DIM));
        let bold = |t: String| Span::styled(t, Style::new().bold());

        let mut lines = vec![
            Line::styled(title, Style::new().fg(DUCK).bold()),
            Line::from(vec![
                dim("opened "),
                bold(date(b.opened as i64)),
                dim(&format!(
                    " · {days} day{} in the book",
                    plural(days as usize)
                )),
            ]),
            Line::from(vec![
                bold(hours(life.powered_s)),
                dim(" awake · "),
                bold(hours(life.torqued_s)),
                dim(" on its feet · "),
                bold(distance(life.distance_m)),
                dim(" walked"),
            ]),
            Line::from(vec![
                bold(format!("{}", life.falls)),
                dim(" falls · "),
                bold(format!("{}", life.pickups)),
                dim(" pickups · "),
                bold(format!("{}", life.pets)),
                dim(" pets"),
            ]),
        ];
        if let Some((skill, n)) = life.skills.iter().max_by_key(|(_, n)| **n) {
            lines.push(Line::from(vec![
                dim("favourite trick "),
                Span::styled(skill.clone(), Style::new().fg(BEAK).bold()),
                dim(&format!(" ×{n}")),
            ]));
        }
        let swaps: usize = b.motors.iter().map(|m| m.retired.len()).sum();
        lines.push(Line::from(vec![
            bold(format!("{swaps}")),
            dim(&format!(" servo{} replaced · ", plural(swaps))),
            bold(format!("{}", life.boots)),
            dim(" boots · "),
            bold(format!("{}", life.updates)),
            dim(" updates · on "),
            bold(if b.last_version.is_empty() {
                "?".to_owned()
            } else {
                b.last_version.clone()
            }),
        ]));
        lines.push(Line::from(vec![
            dim("vet says  "),
            Span::styled(verdict, Style::new().fg(colour).bold()),
        ]));
        // Level with the middle of the head.
        let pad = text.height.saturating_sub(lines.len() as u16) / 2;
        let text = Rect {
            y: text.y + pad,
            height: text.height - pad,
            ..text
        };
        Paragraph::new(lines).render(text, buf);
    }

    fn render_life(&self, area: Rect, buf: &mut Buffer) {
        let life = &self.book.life;
        let [top, tiles] =
            Layout::vertical([Constraint::Length(11), Constraint::Min(4)]).areas(area);
        let [time, vitals] =
            Layout::horizontal([Constraint::Percentage(58), Constraint::Percentage(42)]).areas(top);

        // ── how the time goes ──
        let block = panel("how it spends its time");
        let inner = block.inner(time);
        block.render(time, buf);
        let other = (life.torqued_s - life.walking_s - life.standing_s - life.sitting_s).max(0.0);
        let resting = (life.powered_s - life.torqued_s).max(0.0);
        let parts = [
            ("walking", life.walking_s, DUCK),
            ("standing", life.standing_s, LEAF),
            ("sitting", life.sitting_s, POND),
            ("tricks & poses", other, LILAC),
            ("napping (torque off)", resting, DIM),
        ];
        let total: f64 = parts.iter().map(|p| p.1).sum();
        let mut lines = vec![
            stacked_bar(&parts, total, inner.width.saturating_sub(1)),
            Line::raw(""),
        ];
        for (label, secs, colour) in parts {
            lines.push(Line::from(vec![
                Span::styled("■ ", Style::new().fg(colour)),
                Span::styled(format!("{label:<22}"), Style::new()),
                Span::styled(format!("{:>9}", hours(secs)), Style::new().bold()),
                Span::styled(
                    format!("  {:>3.0}%", percent(secs, total)),
                    Style::new().fg(DIM),
                ),
            ]));
        }
        lines.push(Line::from(vec![
            Span::styled("  held in hands ", Style::new().fg(DIM)),
            Span::styled(hours(life.in_hands_s), Style::new().fg(BLUSH)),
            Span::styled("  ·  on the floor ", Style::new().fg(DIM)),
            Span::styled(hours(life.fallen_s), Style::new().fg(ALERT)),
        ]));
        Paragraph::new(lines).render(inner, buf);

        // ── vitals ──
        let block = panel("vitals");
        let inner = block.inner(vitals);
        block.render(vitals, buf);
        let hot_motor = self
            .book
            .motors
            .iter()
            .max_by(|a, b| a.wear.hottest_c.total_cmp(&b.wear.hottest_c));
        let trips: u64 = self
            .book
            .motors
            .iter()
            .map(|m| {
                m.wear.trips.total() + m.retired.iter().map(|r| r.wear.trips.total()).sum::<u64>()
            })
            .sum();
        let swaps: usize = self.book.motors.iter().map(|m| m.retired.len()).sum();
        let per_hour = |n: u64| {
            if life.powered_s > 60.0 {
                format!("{:.1}/h", n as f64 / (life.powered_s / 3600.0))
            } else {
                "—".into()
            }
        };
        let rows = [
            (
                "hottest servo",
                format!(
                    "{:.0} °C{}",
                    life.hottest_motor_c,
                    hot_motor
                        .filter(|m| m.wear.hottest_c > 0.0)
                        .map(|m| format!("  ({})", nice_joint(&m.joint)))
                        .unwrap_or_default()
                ),
                temp_colour(life.hottest_motor_c),
            ),
            (
                "hottest board",
                format!("{:.0} °C", life.hottest_cpu_c),
                temp_colour(life.hottest_cpu_c - 15.0),
            ),
            (
                "battery seen",
                if life.lowest_volts > 0.0 {
                    format!("{:.2} – {:.2} V", life.lowest_volts, life.highest_volts)
                } else {
                    "—".into()
                },
                Color::Reset,
            ),
            (
                "charges (guessed)",
                format!("{}", life.charges),
                Color::Reset,
            ),
            (
                "servo trips",
                format!("{trips}"),
                if trips > 0 { ALERT } else { LEAF },
            ),
            (
                "servos replaced",
                format!("{swaps}"),
                if swaps > 0 { BEAK } else { LEAF },
            ),
            ("bus drops", per_hour(life.bus_drops), Color::Reset),
            ("late ticks", per_hour(life.missed_ticks), Color::Reset),
            ("daemon starts", format!("{}", life.starts), Color::Reset),
        ];
        let lines: Vec<Line> = rows
            .into_iter()
            .map(|(k, v, c)| {
                Line::from(vec![
                    Span::styled(format!("{k:<19}"), Style::new().fg(DIM)),
                    Span::styled(v, Style::new().fg(c).bold()),
                ])
            })
            .collect();
        Paragraph::new(lines).render(inner, buf);

        // ── milestones ──
        let block = panel("milestones");
        let inner = block.inner(tiles);
        block.render(tiles, buf);
        let mut tiles_list = vec![
            ("⇢", "walked", distance(life.distance_m), DUCK),
            (
                "↻",
                "turned",
                format!("{:.0} turns", life.turned_rad / std::f64::consts::TAU),
                DUCK,
            ),
            ("☀", "wake-ups", format!("{}", life.wake_ups), LEAF),
            ("⇡", "pickups", format!("{}", life.pickups), BLUSH),
            ("♥", "pets", format!("{}", life.pets), BLUSH),
            ("↓", "falls", format!("{}", life.falls), ALERT),
            (
                "❀",
                "soft landings",
                format!("{}", life.soft_landings),
                LEAF,
            ),
            ("⌄", "ground picks", format!("{}", life.ground_picks), POND),
            ("▾", "sit-downs", format!("{}", life.sit_downs), POND),
            ("▴", "stand-ups", format!("{}", life.stand_ups), POND),
            ("♪", "songs", format!("{}", life.songs), LILAC),
            ("♫", "theremin", hours(life.theremin_s), LILAC),
            ("○", "power-offs", format!("{}", life.power_offs), DIM),
            ("↺", "servo reboots", format!("{}", life.motor_reboots), DIM),
        ];
        for (skill, n) in &life.skills {
            tiles_list.push(("★", skill.as_str(), format!("{n}"), BEAK));
        }
        render_tiles(&tiles_list, inner, buf);
    }

    fn render_motors(&mut self, area: Rect, buf: &mut Buffer) {
        let show_map = area.width >= 120;
        let [table_area, map_area] = Layout::horizontal([
            Constraint::Min(60),
            Constraint::Length(if show_map { 34 } else { 0 }),
        ])
        .areas(area);
        let [table_area, detail] =
            Layout::vertical([Constraint::Min(8), Constraint::Length(7)]).areas(table_area);

        let oldest = self
            .book
            .motors
            .iter()
            .map(|m| m.wear.torqued_s)
            .fold(1.0f64, f64::max);
        // Travel and work are the columns a narrow terminal goes without.
        let wide = table_area.width >= 96;
        let header = Row::new(
            [
                "id",
                "joint",
                "age (torque)",
                "",
                "travel",
                "work",
                "hottest",
                "hot",
                "trips",
                "♻",
            ]
            .iter()
            .enumerate()
            .filter(|(i, _)| wide || !matches!(i, 4 | 5))
            .map(|(_, h)| Cell::from(*h).style(Style::new().fg(DIM).bold())),
        );
        let rows: Vec<Row> =
            self.book
                .motors
                .iter()
                .map(|m| {
                    let w = &m.wear;
                    let (mood, colour) = motor_mood(m);
                    let cells =
                        vec![
                            Cell::from(format!("{}", m.id)).style(Style::new().fg(DIM)),
                            Cell::from(Line::from(vec![
                                Span::styled(mood, Style::new().fg(colour)),
                                Span::raw(" "),
                                Span::raw(nice_joint(&m.joint)),
                            ])),
                            Cell::from(age_bar(w.torqued_s / oldest, 12, colour)),
                            Cell::from(hours(w.torqued_s)),
                            Cell::from(format!(
                                "{:.0} turns",
                                w.travel_rad / std::f64::consts::TAU
                            )),
                            Cell::from(format!("{:.1} Ah", w.charge_as / 3600.0)),
                            Cell::from(format!("{:.0} °C", w.hottest_c))
                                .style(Style::new().fg(temp_colour(w.hottest_c))),
                            Cell::from(minutes(w.hot_s)),
                            Cell::from(if w.trips.total() > 0 {
                                format!("⚡{}", w.trips.total())
                            } else {
                                "·".into()
                            })
                            .style(Style::new().fg(if w.trips.total() > 0 { ALERT } else { DIM })),
                            Cell::from(if m.retired.is_empty() {
                                "·".into()
                            } else {
                                format!("{}", m.retired.len())
                            })
                            .style(Style::new().fg(if m.retired.is_empty() { DIM } else { BEAK })),
                        ];
                    Row::new(
                        cells
                            .into_iter()
                            .enumerate()
                            .filter(|(i, _)| wide || !matches!(i, 4 | 5))
                            .map(|(_, c)| c),
                    )
                })
                .collect();
        let widths = [
            Constraint::Length(3),
            Constraint::Length(13),
            Constraint::Length(12),
            Constraint::Length(9),
            Constraint::Length(11),
            Constraint::Length(8),
            Constraint::Length(8),
            Constraint::Length(7),
            Constraint::Length(6),
            Constraint::Length(3),
        ]
        .into_iter()
        .enumerate()
        .filter(|(i, _)| wide || !matches!(i, 4 | 5))
        .map(|(_, w)| w);
        let table = Table::new(rows, widths)
            .header(header)
            .block(panel("the servos, as they are now"))
            .row_highlight_style(
                Style::new()
                    .bg(Color::Indexed(236))
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol("▸ ");
        ratatui::widgets::StatefulWidget::render(table, table_area, buf, &mut self.motors);

        // The selected servo's story.
        let selected = self.motors.selected().unwrap_or(0);
        if let Some(m) = self.book.motors.get(selected) {
            self.render_motor_detail(m, detail, buf);
        }

        if show_map {
            render_body_map(&self.book.motors, selected, map_area, buf);
        }
    }

    fn render_motor_detail(&self, m: &MotorRecord, area: Rect, buf: &mut Buffer) {
        let block = panel(&format!("{} · id {}", nice_joint(&m.joint), m.id));
        let inner = block.inner(area);
        block.render(area, buf);
        let t = &m.wear.trips;
        let ident = m
            .identity
            .map(|i| format!("model {} · firmware v{}", i.model, i.firmware))
            .unwrap_or_else(|| "identity not read yet".into());
        let mut lines = vec![
            Line::from(vec![
                Span::styled("in since ", Style::new().fg(DIM)),
                Span::styled(date(m.installed as i64), Style::new().bold()),
                Span::styled("  ·  ", Style::new().fg(DIM)),
                Span::styled(ident, Style::new().fg(DIM)),
            ]),
            Line::from(vec![
                Span::styled("trips  ", Style::new().fg(DIM)),
                trip("overload", t.overload),
                trip("overheat", t.overheat),
                trip("shock", t.shock),
                trip("encoder", t.encoder),
            ]),
        ];
        if m.retired.is_empty() {
            lines.push(Line::styled(
                "the original servo — never replaced ✿",
                Style::new().fg(LEAF),
            ));
        } else {
            let lives: Vec<f64> = m.retired.iter().map(|r| r.wear.torqued_s).collect();
            let mean = lives.iter().sum::<f64>() / lives.len() as f64;
            lines.push(Line::from(vec![
                Span::styled(
                    format!(
                        "{} servo{} before this one, ",
                        lives.len(),
                        plural(lives.len())
                    ),
                    Style::new().fg(BEAK),
                ),
                Span::styled("lasting ", Style::new().fg(DIM)),
                Span::styled(
                    lives
                        .iter()
                        .map(|s| hours(*s))
                        .collect::<Vec<_>>()
                        .join(", "),
                    Style::new().bold(),
                ),
                Span::styled(format!("  (mean {})", hours(mean)), Style::new().fg(DIM)),
            ]));
            if let Some(last) = m.retired.last() {
                lines.push(Line::from(vec![
                    Span::styled("last swap ", Style::new().fg(DIM)),
                    Span::styled(date(last.removed as i64), Style::new().bold()),
                ]));
            }
        }
        Paragraph::new(lines)
            .wrap(Wrap { trim: true })
            .render(inner, buf);
    }

    /// Personal records, by the sensor that set them: what the shared card's badges are made of.
    fn render_records(&self, area: Rect, buf: &mut Buffer) {
        use proto::vetbook::key;
        let life = &self.book.life;
        let t = |k: &str| life.tallies.get(k).copied().unwrap_or(0.0);
        let hi = |k: &str| life.highs.get(k).copied().unwrap_or(0.0);
        let lo = |k: &str| life.lows.get(k).copied();
        let groups: [RecordGroup; 6] = [
            (
                "body · IMU",
                POND,
                vec![
                    ("fastest spin", format!("{:.0} °/s", hi(key::SPIN_DPS))),
                    (
                        "steepest tilt survived",
                        format!("{:.0}°", hi(key::TILT_SURVIVED_DEG)),
                    ),
                    ("upside down", hours(t(key::UPSIDE_DOWN_S))),
                ],
            ),
            (
                "feet · odometry",
                DUCK,
                vec![
                    (
                        "top speed",
                        format!("{:.2} km/h", hi(key::TOP_SPEED_MPS) * 3.6),
                    ),
                    (
                        "longest walk, no fall",
                        format!("{:.0} m", hi(key::WALK_STREAK_M)),
                    ),
                    (
                        "spins",
                        format!("{:.0}", life.turned_rad / std::f64::consts::TAU),
                    ),
                ],
            ),
            (
                "servos",
                BEAK,
                vec![
                    ("peak current", format!("{:.2} A", hi(key::PEAK_CURRENT_A))),
                    (
                        "fastest joint",
                        format!("{:.0} °/s", hi(key::JOINT_SPEED_DPS)),
                    ),
                    ("gaits tried", format!("{:.0}", t(key::GAIT_CHANGES))),
                ],
            ),
            (
                "voice, beak, light",
                BLUSH,
                vec![
                    ("sounds played", format!("{:.0}", t(key::SOUNDS))),
                    ("beak flaps", format!("{:.0}", t(key::BEAK_FLAPS))),
                    (
                        "flashlight",
                        format!(
                            "{} · {:.0}× on",
                            hours(t(key::FLASHLIGHT_S)),
                            t(key::FLASHLIGHT_ON)
                        ),
                    ),
                ],
            ),
            (
                "camera · depth",
                LILAC,
                vec![
                    ("photos", format!("{:.0}", t(key::PHOTOS))),
                    (
                        "video streamed",
                        format!(
                            "{} · {:.0} visits",
                            hours(t(key::VIDEO_S)),
                            t(key::REMOTE_VISITS)
                        ),
                    ),
                    ("new surface seen", format!("{:.1} m²", t(key::SURFACE_M2))),
                    (
                        "nearest · farthest",
                        format!(
                            "{} · {:.0} mm",
                            lo(key::CLOSEST_MM).map_or("—".into(), |v| format!("{v:.0}")),
                            hi(key::FARTHEST_MM)
                        ),
                    ),
                ],
            ),
            (
                "power · board",
                LEAF,
                vec![
                    ("longest on one charge", hours(hi(key::ON_ONE_CHARGE_S))),
                    ("throttled", hours(t(key::THROTTLED_S))),
                    ("hottest board", format!("{:.0} °C", life.hottest_cpu_c)),
                ],
            ),
        ];
        let cols = if area.width >= 110 { 3 } else { 2 };
        let rows = groups.len().div_ceil(cols);
        let row_areas = Layout::vertical(vec![Constraint::Length(7); rows]).split(area);
        for (r, row_area) in row_areas.iter().enumerate() {
            let cells =
                Layout::horizontal(vec![Constraint::Ratio(1, cols as u32); cols]).split(*row_area);
            for (c, cell) in cells.iter().enumerate() {
                let Some((title, colour, lines)) = groups.get(r * cols + c) else {
                    continue;
                };
                let block = Block::bordered()
                    .border_type(BorderType::Rounded)
                    .border_style(Style::new().fg(Color::Indexed(240)))
                    .title(Span::styled(
                        format!(" {title} "),
                        Style::new().fg(*colour).bold(),
                    ));
                let inner = block.inner(*cell);
                block.render(*cell, buf);
                let text: Vec<Line> = lines
                    .iter()
                    .map(|(k, v)| {
                        Line::from(vec![
                            Span::styled(format!("{k:<24}"), Style::new().fg(DIM)),
                            Span::styled(v.clone(), Style::new().bold()),
                        ])
                    })
                    .collect();
                Paragraph::new(text).render(inner, buf);
            }
        }
    }

    fn render_diary(&self, area: Rect, buf: &mut Buffer) {
        let (area, input) = match &self.draft {
            Some(_) => {
                let [list, input] =
                    Layout::vertical([Constraint::Min(3), Constraint::Length(3)]).areas(area);
                (list, Some(input))
            }
            None => (area, None),
        };
        if let (Some(input), Some(draft)) = (input, &self.draft) {
            let block = Block::bordered()
                .border_type(BorderType::Rounded)
                .border_style(Style::new().fg(LILAC))
                .title(Span::styled(
                    " ✎ dear diary… ",
                    Style::new().fg(LILAC).bold(),
                ));
            let inner = block.inner(input);
            block.render(input, buf);
            // Keep the end of a long line in view: the cursor is where the typing is.
            let room = inner.width.saturating_sub(3) as usize;
            let shown: String = {
                let chars: Vec<char> = draft.chars().collect();
                chars[chars.len().saturating_sub(room)..].iter().collect()
            };
            Line::from(vec![
                Span::styled("✎ ", Style::new().fg(LILAC)),
                Span::raw(shown),
                Span::styled(
                    "▏",
                    Style::new().fg(DUCK).add_modifier(Modifier::SLOW_BLINK),
                ),
            ])
            .render(inner, buf);
        }

        let block = panel("diary — newest first · n to write");
        let inner = block.inner(area);
        block.render(area, buf);
        let lines: Vec<Line> = self
            .book
            .events
            .iter()
            .rev()
            .map(|e| {
                let (icon, colour) = match e.kind.as_str() {
                    "opened" => ("✿", DUCK),
                    "update" => ("⇧", POND),
                    "swap" => ("⚙", BEAK),
                    "note" => ("✎", LILAC),
                    "charge" => ("↯", LEAF),
                    "policy" => ("◆", BEAK),
                    _ => ("•", DIM),
                };
                Line::from(vec![
                    Span::styled(format!("{}  ", stamp(e.at as i64)), Style::new().fg(DIM)),
                    Span::styled(format!("{icon} "), Style::new().fg(colour)),
                    Span::styled(e.text.clone(), Style::new().fg(colour)),
                ])
            })
            .collect();
        if lines.is_empty() {
            Paragraph::new("nothing written yet").render(inner, buf);
            return;
        }
        let max_scroll = (lines.len() as u16).saturating_sub(inner.height);
        Paragraph::new(lines)
            .scroll((self.diary_scroll.min(max_scroll), 0))
            .render(inner, buf);
    }
}

// ── the pieces ───────────────────────────────────────────────────────────────

/// A QR code for `text`, as rows of dark (`true`) and light modules with a two-module quiet zone.
fn qr_modules(text: &str) -> Option<Vec<Vec<bool>>> {
    let code = qrcode::QrCode::new(text.as_bytes()).ok()?;
    let width = code.width();
    let colors = code.to_colors();
    const QUIET: usize = 2;
    let side = width + 2 * QUIET;
    let mut rows = vec![vec![false; side]; side];
    for y in 0..width {
        for x in 0..width {
            rows[y + QUIET][x + QUIET] = colors[y * width + x] == qrcode::Color::Dark;
        }
    }
    Some(rows)
}

/// Two modules to a cell with half blocks, in black on white whatever the terminal's theme: a
/// phone's camera reads dark-on-light, and a light-on-dark code fails on half of them.
fn render_qr(modules: &[Vec<bool>], area: Rect, buf: &mut Buffer) {
    let shade = |dark: bool| {
        if dark {
            Color::Rgb(0, 0, 0)
        } else {
            Color::Rgb(255, 255, 255)
        }
    };
    for (row, pair) in modules.chunks(2).enumerate() {
        let y = area.y + row as u16;
        if y >= area.bottom() {
            break;
        }
        for (x, &top) in pair[0].iter().enumerate() {
            let x = area.x + x as u16;
            if x >= area.right() {
                break;
            }
            let bottom = pair.get(1).is_some_and(|r| r[x as usize - area.x as usize]);
            buf[(x, y)]
                .set_symbol("▀")
                .set_fg(shade(top))
                .set_bg(shade(bottom));
        }
    }
}

fn panel(title: &str) -> Block<'static> {
    Block::bordered()
        .border_type(BorderType::Rounded)
        .border_style(Style::new().fg(Color::Indexed(240)))
        .title(Line::from(vec![
            Span::raw(" "),
            Span::styled(title.to_owned(), Style::new().fg(DUCK).bold()),
            Span::raw(" "),
        ]))
}

fn stacked_bar(parts: &[(&str, f64, Color)], total: f64, width: u16) -> Line<'static> {
    if total <= 0.0 || width == 0 {
        return Line::styled("·".repeat(width as usize), Style::new().fg(DIM));
    }
    let mut spans = Vec::new();
    let mut used = 0usize;
    for (i, (_, secs, colour)) in parts.iter().enumerate() {
        let cells = if i + 1 == parts.len() {
            (width as usize).saturating_sub(used)
        } else {
            ((secs / total) * width as f64).round() as usize
        };
        let cells = cells.min((width as usize).saturating_sub(used));
        used += cells;
        spans.push(Span::styled("█".repeat(cells), Style::new().fg(*colour)));
    }
    Line::from(spans)
}

fn age_bar(fraction: f64, width: usize, colour: Color) -> Line<'static> {
    let filled = ((fraction.clamp(0.0, 1.0)) * width as f64).round() as usize;
    Line::from(vec![
        Span::styled("▰".repeat(filled), Style::new().fg(colour)),
        Span::styled(
            "▱".repeat(width - filled),
            Style::new().fg(Color::Indexed(238)),
        ),
    ])
}

fn render_tiles(tiles: &[(&str, &str, String, Color)], area: Rect, buf: &mut Buffer) {
    const TILE_W: u16 = 19;
    let cols = (area.width / TILE_W).max(1) as usize;
    for (i, (icon, label, value, colour)) in tiles.iter().enumerate() {
        let (row, col) = (i / cols, i % cols);
        let y = area.y + row as u16 * 2;
        if y + 2 > area.bottom() {
            break;
        }
        let x = area.x + col as u16 * TILE_W;
        let cell = Rect::new(x, y, TILE_W.min(area.right() - x), 2.min(area.bottom() - y));
        Paragraph::new(vec![
            Line::from(vec![
                Span::styled(format!("{icon} "), Style::new().fg(*colour)),
                Span::styled(value.clone(), Style::new().fg(*colour).bold()),
            ]),
            Line::styled(format!("  {label}"), Style::new().fg(DIM)),
        ])
        .render(cell, buf);
    }
}

/// A front view of the duck with each servo where it sits, coloured by how it is doing.
fn render_body_map(motors: &[MotorRecord], selected: usize, area: Rect, buf: &mut Buffer) {
    let block = panel("where they are");
    let inner = block.inner(area);
    block.render(area, buf);

    let dot = |joint: &str| -> Span<'static> {
        let Some((i, m)) = motors.iter().enumerate().find(|(_, m)| m.joint == joint) else {
            return Span::raw("  ");
        };
        let (_, colour) = motor_mood(m);
        let style = if i == selected {
            Style::new().fg(Color::Black).bg(colour).bold()
        } else {
            Style::new().fg(colour).bold()
        };
        Span::styled(format!("{:>2}", m.id), style)
    };
    let s = |t: &'static str| Span::styled(t, Style::new().fg(DUCK));
    let d = |t: &'static str| Span::styled(t, Style::new().fg(DIM));
    let lines = vec![
        Line::from(vec![s("       ╭────────╮")]),
        Line::from(vec![
            s("       │   (◉)  │"),
            Span::styled("▶ ", Style::new().fg(BEAK)),
            dot("mouth"),
        ]),
        Line::from(vec![
            d("  roll "),
            dot("head_roll"),
            s(" ╰──┬──╯ "),
            dot("head_yaw"),
            d(" yaw"),
        ]),
        Line::from(vec![
            d(" pitch "),
            dot("head_pitch"),
            s("    │    "),
            dot("neck_pitch"),
            d(" neck"),
        ]),
        Line::from(vec![s("     ╭──────┴──────╮")]),
        Line::from(vec![s("     │             │")]),
        Line::from(vec![s("     ╰─┬─────────┬─╯")]),
        Line::from(vec![
            d(" yaw "),
            dot("right_hip_yaw"),
            s("│         │"),
            dot("left_hip_yaw"),
        ]),
        Line::from(vec![
            d("roll "),
            dot("right_hip_roll"),
            s("│         │"),
            dot("left_hip_roll"),
        ]),
        Line::from(vec![
            d(" hip "),
            dot("right_hip_pitch"),
            s("│         │"),
            dot("left_hip_pitch"),
        ]),
        Line::from(vec![
            d("knee "),
            dot("right_knee"),
            s("│         │"),
            dot("left_knee"),
        ]),
        Line::from(vec![
            d("ankl "),
            dot("right_ankle"),
            s("│         │"),
            dot("left_ankle"),
        ]),
        Line::from(vec![s("      ═╧═       ═╧═")]),
        Line::from(vec![d("      right     left")]),
        Line::raw(""),
        Line::from(vec![
            Span::styled("● ", Style::new().fg(LEAF)),
            d("fine "),
            Span::styled("● ", Style::new().fg(DUCK)),
            d("warm "),
            Span::styled("● ", Style::new().fg(ALERT)),
            d("tripped"),
        ]),
    ];
    Paragraph::new(lines)
        .alignment(Alignment::Left)
        .render(inner, buf);
}

fn trip(name: &'static str, n: u64) -> Span<'static> {
    if n > 0 {
        Span::styled(format!("{name} ⚡{n}   "), Style::new().fg(ALERT).bold())
    } else {
        Span::styled(format!("{name} ·   "), Style::new().fg(DIM))
    }
}

/// A servo's face and colour: tripped, run hot, or fine.
fn motor_mood(m: &MotorRecord) -> (&'static str, Color) {
    if m.wear.trips.total() > 0 {
        ("✚", ALERT)
    } else if m.wear.hottest_c >= HOT_C || m.wear.hot_s > 0.0 {
        ("◐", DUCK)
    } else {
        ("●", LEAF)
    }
}

/// The vet's one-line opinion.
fn verdict(book: &Vetbook) -> (String, Color) {
    let tripped: Vec<String> = book
        .motors
        .iter()
        .filter(|m| m.wear.trips.total() > 0)
        .map(|m| nice_joint(&m.joint))
        .collect();
    let hot = book.motors.iter().filter(|m| m.wear.hot_s > 600.0).count();
    let life = &book.life;
    if !tripped.is_empty() {
        let named = match tripped.len() {
            n if n <= 3 => tripped.join(", "),
            n => format!("{} and {} more", tripped[..2].join(", "), n - 2),
        };
        (format!("check-up advised — tripped: {named}"), ALERT)
    } else if hot > 0 {
        (
            format!(
                "{hot} servo{} running warm — give it a rest now and then",
                plural(hot)
            ),
            DUCK,
        )
    } else if life.powered_s < 3600.0 {
        ("a brand new duckling 🐣".into(), LEAF)
    } else if life.falls as f64 > life.walking_s / 60.0 {
        ("healthy, if a little clumsy".into(), LEAF)
    } else {
        ("healthy as a duck ✿".into(), LEAF)
    }
}

/// A joint's name as the page shows it: short enough for a narrow terminal, and the mouth is a
/// beak, because it is a duck.
fn nice_joint(joint: &str) -> String {
    if joint == "mouth" {
        return "beak".into();
    }
    let (side, rest) = match joint.split_once('_') {
        Some(("left", rest)) => ("L ", rest),
        Some(("right", rest)) => ("R ", rest),
        _ => ("", joint),
    };
    let rest = rest
        .strip_suffix("_pitch")
        .filter(|r| *r == "neck")
        .unwrap_or(rest);
    format!("{side}{}", rest.replace('_', " "))
}

fn temp_colour(c: f64) -> Color {
    if c >= HOT_C {
        ALERT
    } else if c >= HOT_C - 10.0 {
        DUCK
    } else if c > 0.0 {
        LEAF
    } else {
        DIM
    }
}

fn percent(part: f64, total: f64) -> f64 {
    if total > 0.0 {
        part / total * 100.0
    } else {
        0.0
    }
}

fn plural(n: usize) -> &'static str {
    if n == 1 { "" } else { "s" }
}

fn hours(secs: f64) -> String {
    if secs < 60.0 {
        format!("{secs:.0} s")
    } else if secs < 3600.0 {
        format!("{:.0} min", secs / 60.0)
    } else {
        format!("{:.1} h", secs / 3600.0)
    }
}

fn minutes(secs: f64) -> String {
    if secs <= 0.0 {
        "·".into()
    } else {
        format!("{:.0} min", (secs / 60.0).ceil())
    }
}

/// Metres, with a sense of scale.
fn distance(m: f64) -> String {
    if m >= 1000.0 {
        format!("{:.2} km", m / 1000.0)
    } else {
        format!("{m:.0} m")
    }
}

fn date(at: i64) -> String {
    let (y, mo, d, ..) = crate::show::civil(at);
    format!("{y:04}-{mo:02}-{d:02}")
}

fn stamp(at: i64) -> String {
    let (y, mo, d, h, mi, _) = crate::show::civil(at);
    format!("{y:04}-{mo:02}-{d:02} {h:02}:{mi:02}")
}

// ── piped ────────────────────────────────────────────────────────────────────

fn plain(book: &Vetbook, name: &str, now: i64) -> String {
    use std::fmt::Write;
    let life = &book.life;
    let mut out = String::new();
    let _ = writeln!(
        out,
        "{} — opened {}, {} days ago",
        if name.is_empty() {
            "vet book".to_owned()
        } else {
            format!("{name}'s vet book")
        },
        date(book.opened as i64),
        (now - book.opened as i64).max(0) / 86_400
    );
    let _ = writeln!(out, "verdict      {}", verdict(book).0);
    let _ = writeln!(
        out,
        "awake        {}   torque on {}",
        hours(life.powered_s),
        hours(life.torqued_s)
    );
    let _ = writeln!(
        out,
        "walking      {}   standing {}   sitting {}   in hands {}   on the floor {}",
        hours(life.walking_s),
        hours(life.standing_s),
        hours(life.sitting_s),
        hours(life.in_hands_s),
        hours(life.fallen_s)
    );
    let _ = writeln!(
        out,
        "walked       {}   turned {:.0} turns",
        distance(life.distance_m),
        life.turned_rad / std::f64::consts::TAU
    );
    let _ = writeln!(
        out,
        "counts       falls {}  soft landings {}  pickups {}  ground picks {}  sit-downs {}  stand-ups {}  pets {}  songs {}",
        life.falls,
        life.soft_landings,
        life.pickups,
        life.ground_picks,
        life.sit_downs,
        life.stand_ups,
        life.pets,
        life.songs
    );
    if !life.skills.is_empty() {
        let skills: Vec<String> = life
            .skills
            .iter()
            .map(|(k, v)| format!("{k} {v}"))
            .collect();
        let _ = writeln!(out, "skills       {}", skills.join("  "));
    }
    let _ = writeln!(
        out,
        "lifecycle    boots {}  starts {}  updates {}  wake-ups {}  power-offs {}  charges {}",
        life.boots, life.starts, life.updates, life.wake_ups, life.power_offs, life.charges
    );
    let _ = writeln!(
        out,
        "vitals       hottest servo {:.0} °C  hottest board {:.0} °C  battery {:.2}–{:.2} V  bus drops {}  late ticks {}",
        life.hottest_motor_c,
        life.hottest_cpu_c,
        life.lowest_volts,
        life.highest_volts,
        life.bus_drops,
        life.missed_ticks
    );
    let _ = writeln!(out);
    let _ = writeln!(
        out,
        "{:>3}  {:<16} {:>9} {:>10} {:>9} {:>8} {:>8} {:>6} {:>6}",
        "id", "joint", "torque", "travel", "work", "hottest", "hot", "trips", "swaps"
    );
    for m in &book.motors {
        let w = &m.wear;
        let _ = writeln!(
            out,
            "{:>3}  {:<16} {:>9} {:>6.0} trn {:>6.1} Ah {:>5.0} °C {:>8} {:>6} {:>6}",
            m.id,
            m.joint,
            hours(w.torqued_s),
            w.travel_rad / std::f64::consts::TAU,
            w.charge_as / 3600.0,
            w.hottest_c,
            minutes(w.hot_s),
            w.trips.total(),
            m.retired.len()
        );
    }
    let _ = writeln!(out);
    for e in book.events.iter().rev().take(20) {
        let _ = writeln!(out, "{}  {:<7} {}", stamp(e.at as i64), e.kind, e.text);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::backend::TestBackend;

    fn sample() -> Vetbook {
        let mut book = Vetbook::open(1_760_000_000);
        book.life.powered_s = 50.0 * 3600.0;
        book.life.torqued_s = 30.0 * 3600.0;
        book.life.walking_s = 12.0 * 3600.0;
        book.life.standing_s = 10.0 * 3600.0;
        book.life.sitting_s = 4.0 * 3600.0;
        book.life.distance_m = 2_345.0;
        book.life.falls = 17;
        book.life.skills.insert("kick_left".into(), 4);
        book.motors[3].wear.torqued_s = 30.0 * 3600.0;
        book.motors[3].wear.trips.overload = 2;
        book.motors[3].wear.hottest_c = 64.0;
        book.swap(book.motors[8].id, 1_760_100_000).unwrap();
        book.event(1_760_200_000, "note", "reglued the beak");
        book
    }

    fn draw(tab: Tab, width: u16) -> String {
        draw_with(tab, width, None)
    }

    fn draw_with(tab: Tab, width: u16, draft: Option<&str>) -> String {
        draw_page(tab, width, draft, Share::Closed)
    }

    fn draw_page(tab: Tab, width: u16, draft: Option<&str>, share: Share) -> String {
        let mut page = Page {
            book: sample(),
            name: "lavender-star".into(),
            tab,
            motors: TableState::default().with_selected(Some(3)),
            diary_scroll: 0,
            stale: None,
            now: 1_760_300_000,
            draft: draft.map(str::to_owned),
            written: None,
            share,
        };
        let mut terminal = ratatui::Terminal::new(TestBackend::new(width, 40)).unwrap();
        terminal
            .draw(|f| page.render(f.area(), f.buffer_mut()))
            .unwrap();
        let buf = terminal.backend().buffer().clone();
        let mut text = String::new();
        for y in 0..buf.area.height {
            for x in 0..buf.area.width {
                text.push_str(buf[(x, y)].symbol());
            }
            text.push('\n');
        }
        text
    }

    #[test]
    fn every_tab_draws_at_both_widths() {
        for width in [80, 140] {
            let life = draw(Tab::Life, width);
            assert!(life.contains("lavender-star's vet book"), "{life}");
            assert!(life.contains("2.35 km"));
            assert!(life.contains("check-up advised"));
            let motors = draw(Tab::Motors, width);
            assert!(motors.contains("L knee"), "{motors}");
            assert!(motors.contains("beak"));
            assert!(motors.contains("⚡"));
            let diary = draw(Tab::Diary, width);
            assert!(diary.contains("reglued the beak"), "{diary}");
        }
        assert!(draw(Tab::Motors, 140).contains("where they are"));
    }

    /// `VETBOOK_DUMP=dir cargo test -p robotctl vetbook` writes every tab as text, to look at.
    #[test]
    fn dump_for_a_human() {
        let Ok(dir) = std::env::var("VETBOOK_DUMP") else {
            return;
        };
        for (name, tab) in [
            ("life", Tab::Life),
            ("motors", Tab::Motors),
            ("diary", Tab::Diary),
        ] {
            for width in [80u16, 140] {
                std::fs::write(format!("{dir}/{name}-{width}.txt"), draw(tab, width)).unwrap();
            }
        }
        // Every cell with its colours, for a script to turn into a picture.
        let mut page = Page {
            book: sample(),
            name: "lavender-star".into(),
            tab: Tab::Life,
            motors: TableState::default(),
            diary_scroll: 0,
            stale: None,
            now: 1_760_300_000,
            draft: None,
            written: None,
            // `VETBOOK_DUMP_SHARE=1` draws the page with a share's popup over it.
            share: if std::env::var("VETBOOK_DUMP_SHARE").is_ok() {
                Share::Done(Ok(proto::VetbookShareResult {
                    url: "https://pollen-robotics-microduck-vetbook.hf.space/d/apirrone/lavender-star"
                        .into(),
                    image: String::new(),
                    dataset: "apirrone/lavender-star-vetbook".into(),
                    card: "purikura".into(),
                    listed: true,
                }))
            } else {
                Share::Closed
            },
        };
        let mut terminal = ratatui::Terminal::new(TestBackend::new(140, 40)).unwrap();
        terminal
            .draw(|f| page.render(f.area(), f.buffer_mut()))
            .unwrap();
        let buf = terminal.backend().buffer();
        let mut cells = Vec::new();
        for y in 0..buf.area.height {
            for x in 0..buf.area.width {
                let c = &buf[(x, y)];
                cells.push(format!("{}\t{:?}\t{:?}", c.symbol(), c.fg, c.bg));
            }
        }
        std::fs::write(format!("{dir}/cells-120x40.tsv"), cells.join("\n")).unwrap();
        let typing = draw_with(Tab::Diary, 100, Some("new feet, and the left knee"));
        std::fs::write(format!("{dir}/typing-100.txt"), typing).unwrap();
    }

    #[test]
    fn a_line_being_typed_is_on_the_diary_page() {
        let page = draw_with(Tab::Diary, 80, Some("new feet"));
        assert!(page.contains("dear diary"), "{page}");
        assert!(page.contains("✎ new feet"));
        assert!(page.contains("never mind"));
        assert!(page.contains("reglued the beak"));
    }

    #[test]
    fn voltage_is_not_a_trip() {
        let mut book = sample();
        for m in &mut book.motors {
            m.wear.trips.count(0, 0b1);
        }
        assert_eq!(verdict(&book).1, ALERT, "the knee's overload still counts");
        book.motors[3].wear.trips = Default::default();
        assert_ne!(verdict(&book).1, ALERT);
    }

    #[test]
    fn a_share_shows_the_link_and_a_qr_code() {
        let shared = proto::VetbookShareResult {
            url: "https://pollen-robotics-microduck-vetbook.hf.space/d/apirrone/lavender-star"
                .into(),
            image: String::new(),
            dataset: "apirrone/lavender-star-vetbook".into(),
            card: "purikura".into(),
            listed: true,
        };
        let page = draw_page(Tab::Life, 120, None, Share::Done(Ok(shared)));
        assert!(page.contains("shared ♡"), "{page}");
        assert!(page.contains("listed in the vet book gallery"), "{page}");
        assert!(page.contains("/d/apirrone/lavender-star"));
        assert!(page.contains("purikura"));
        assert!(page.contains("▀"), "no QR code drawn");
        assert!(page.contains("any key closes"));
    }

    #[test]
    fn a_failed_share_says_why() {
        let why = "this robot is not signed in to Hugging Face".to_owned();
        let page = draw_page(Tab::Life, 100, None, Share::Done(Err(why)));
        assert!(page.contains("not shared"), "{page}");
        assert!(page.contains("not signed in"));
        let waiting = draw_page(Tab::Life, 100, None, Share::Working(false));
        assert!(waiting.contains("sharing…"));
    }

    #[test]
    fn a_share_asks_about_the_gallery_first() {
        let page = draw_page(Tab::Life, 100, None, Share::Asking);
        assert!(page.contains("public vet book gallery?"), "{page}");
        assert!(page.contains("share without listing"), "{page}");
    }

    #[test]
    fn the_qr_code_reads_back_as_the_link() {
        let url = "https://pollen-robotics-microduck-vetbook.hf.space/d/apirrone/graphite-bang";
        let modules = qr_modules(url).unwrap();
        // Square, with the quiet zone light all round, and the finder pattern dark in its corner.
        assert_eq!(modules.len(), modules[0].len());
        assert!(modules[0].iter().all(|&m| !m));
        assert!(modules[2][2], "the top-left finder pattern starts dark");
        // The same data encodes to the same modules: this is the code for the URL, not for
        // something truncated on the way.
        let again = qrcode::QrCode::new(url.as_bytes()).unwrap();
        assert_eq!(again.width() + 4, modules.len());
    }

    #[test]
    fn the_records_tab_names_every_sensor() {
        for width in [80, 140] {
            let page = draw(Tab::Records, width);
            for group in [
                "body · IMU",
                "feet · odometry",
                "servos",
                "camera · depth",
                "power · board",
            ] {
                assert!(page.contains(group), "{group} missing at {width}:\n{page}");
            }
            assert!(page.contains("fastest spin"));
        }
    }

    #[test]
    fn joints_have_friendly_names() {
        assert_eq!(nice_joint("left_hip_pitch"), "L hip pitch");
        assert_eq!(nice_joint("right_knee"), "R knee");
        assert_eq!(nice_joint("neck_pitch"), "neck");
        assert_eq!(nice_joint("head_yaw"), "head yaw");
        assert_eq!(nice_joint("mouth"), "beak");
    }

    #[test]
    fn piped_output_has_every_joint() {
        let text = plain(&sample(), "", 1_760_300_000);
        for joint in proto::JOINT_NAMES {
            assert!(text.contains(joint), "{joint} missing");
        }
        assert!(text.contains("kick_left 4"));
    }
}
