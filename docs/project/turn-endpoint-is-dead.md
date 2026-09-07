# The TURN endpoint is dead, and the delegation behind it is dangling

Recorded 2026-09-07, from
[`reachy_mini` #1356](https://github.com/pollen-robotics/reachy_mini/issues/1356). Every claim
below was re-checked from this machine on that date.

`mediad` fetches relay credentials from `https://turn.fastrtc.org/credentials`
(`mediad/src/turn.rs`, `DEFAULT_TURN_ENDPOINT`), and `mediad/systemd/mediad.service` passes no
`--turn-url` — so that default is what every microduck robot in the fleet uses. It has not
resolved since at least 2026-06-04, which is when
[gradio-app/fastrtc#429](https://github.com/gradio-app/fastrtc/issues/429) was filed. It is still
open, with two "me too" comments and no answer; `fastrtc`'s last push was 2026-01-12, and
`backend/fastrtc/credentials.py` still points at the same host.

## It is a deleted hosted zone, not a lapsed registration

```bash
dig turn.fastrtc.org @1.1.1.1
```

`SERVFAIL`. The `.org` registry does delegate the zone — four Route53 nameservers:

```bash
dig NS fastrtc.org @a0.org.afilias-nst.info +norecurse
```

```
fastrtc.org.  3600  IN  NS  ns-79.awsdns-09.com.
fastrtc.org.  3600  IN  NS  ns-974.awsdns-57.net.
fastrtc.org.  3600  IN  NS  ns-1297.awsdns-34.org.
fastrtc.org.  3600  IN  NS  ns-1797.awsdns-32.co.uk.
```

All four answer `REFUSED` for the zone they are named as authoritative for, which is what a
Route53 nameserver says when the hosted zone behind it no longer exists:

```bash
dig SOA fastrtc.org @ns-79.awsdns-09.com +norecurse
```

The registration itself is healthy — created 2025-02-17, expiring 2027-02-17, last updated
2026-08-12, and carrying all four `client*Prohibited` locks:

```bash
whois fastrtc.org
```

So somebody at `fastrtc` still holds the domain and could restore the zone in minutes. Nobody has
in three months.

This corrects `remote-access-design.md` §6, which recorded "`fastrtc.org` has no NS records at
all" and read the outage as "a lapsed registration or an outage rather than a moved URL". The
delegation is present and the registration is live; what is missing is the zone the delegation
points at. The difference is not pedantic — it is the difference between waiting for an outage to
end and sitting downstream of a dangling delegation.

## What it costs this fleet

**A signed-in robot offers its account token to whoever answers for that name.** A dangling
Route53 delegation is a known takeover route: create hosted zones for `fastrtc.org` until AWS
assigns one of the four delegated nameservers, and *one* is enough, because a resolver needs only
one authoritative server to answer. Whoever lands it serves records for the name, passes DNS
validation for a certificate on it, and then `turn.rs`'s `fetch` — `bearer_auth(token)` on a
`GET`, every thirty seconds, from every signed-in robot — hands over the Hugging Face access
token `updaterd` wrote. That token is the robot's whole account credential: §2.4 of
`remote-access-design.md` is about how broad its scopes are.

The redirect half of the same problem does not apply here. `reachy_mini` #1364 found bearer-
carrying requests following redirects; `reqwest` 0.13.4 strips `Authorization` when a redirect
crosses scheme, host or port (`src/redirect.rs`, `remove_sensitive_headers`), so `mediad` cannot
be walked to a third-party origin that way. The exposure is same-origin, and same-origin needs no
redirect.

**No relay candidate is ever offered.** Host and srflx only, on both ends. A consumer behind
symmetric NAT, CGNAT, or a firewall that drops UDP cannot reach a robot at all, and there is no
fallback because the fallback is the thing that is down. Demos pass regardless: two peers on one
network pair on host candidates and never look at a relay, which is exactly why this went
unnoticed through #1182 and since.

**A warning every thirty seconds, for the life of the daemon.** `turn.rs` treats every failed
fetch as transient and retries after `RETRY_AFTER_FAILURE` — 30 s — logging `could not fetch relay
credentials` each time. With `Environment=RUST_LOG=info` on the unit and journald persistent at
three months' retention, that is about 2,880 lines a day of one message that will never come true,
crowding out the history somebody reads while diagnosing something else. §6 of
`remote-access-design.md` names this cost precisely — "a warning every thirty seconds for the life
of the daemon is how a log stops being read" — and then spends it, because the case it guards
against is the robot with no token. Nothing in the module distinguishes a proxy that hiccuped from
a name that will not resolve today and will not resolve tomorrow.

## Where `reachy_mini` stands on it

- **#1182** introduced the default. The endpoint was already dead when it merged.
- Branch **`no-default-turn-url`** (one commit, `20553846`) drops it: `REACHY_TURN_URL` defaults
  to `""`, the refresher's `start()` becomes a no-op that says so once, and the docs stop
  advertising the host. No PR is open for it, and it is 20 commits behind `main`.
- **#1364 / PR #1365** (both open) require every endpoint that receives the daemon token to be
  `https`, free of credentials, query and fragment, and refuse redirects on those requests — TURN
  included. That closes the untrusted-destination gap and leaves `turn.fastrtc.org` as the
  default, so the outcome is a validated dead endpoint.

Neither is merged, so nothing has shipped on either side.

## What to do here

**Drop the default, so a relay is opt-in.** `--turn-url` stays as the seam; with no value,
`maintain` does not spawn and says once that a relay is off. This removes the token exfiltration
route, removes the log spam, and costs no relay coverage, because the coverage is already zero —
the endpoint that would provide it does not answer. It is worth being clear that this is not a
workaround for a broken dependency: the dependency is not coming back on its own, and the default
is actively harmful while it stands.

**Our own credentials proxy, as its own piece of work.** This is what that endpoint *is*: a small
service holding a Cloudflare Calls key and minting short-lived credentials for a caller presenting
a valid Hugging Face token. It restores relay coverage, it keeps the key in one place instead of
on every board, and `--turn-url` is already where it plugs in. §6 of `remote-access-design.md`
already reached this conclusion; what has changed is that "wait, having reported it" is no longer
the cheap first option, because waiting now means leaving the token exposed.

**A Cloudflare key on each robot** — `TURN_KEY_ID` and `TURN_KEY_API_TOKEN` straight on the board
— remains the wrong answer for the reason §2.4 gives: a long-lived API token on every unit.

**And a relay-only connectivity check**, so a default endpoint that answers nothing cannot merge
silently a second time. Every existing check passes today, because every existing check runs
between two peers that pair on host candidates.

## Also worth doing, outside this repo

Tell whoever owns `fastrtc`. #429 has been open since June with no maintainer reply, and the fix
on their side is recreating one hosted zone. Until then anybody following their documentation ships
the same dead default, and the `.org` delegation stays takeover-shaped for the rest of the fleet
that trusts it.
