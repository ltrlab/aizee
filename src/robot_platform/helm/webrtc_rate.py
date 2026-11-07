"""The video's rate on a remote viewer's link: the lower of what the receiver's delay estimate allows (its REMB) and what
the loss allows, on the rule of Google Congestion Control (draft-ietf-rmcat-gcc): above 10 % lost, down by half the
loss; below 2 %, up 5 %; between, as it is. Near the rate that last lost too much it climbs slowly, as a carrier's cap
stays where it is: climbing back through it at 5 % a report freezes the picture each time. No imports beyond Python's:
scripts/webrtc_link_probe.py runs it anywhere."""

LOSS_DOWN, LOSS_UP = 0.10, 0.02
GROWTH, CAUTIOUS_GROWTH = 1.05, 1.005
NEAR_CEILING = 0.85  # Above this share of the rate that last lost too much, it climbs cautiously,
FORGET_REPORTS = 60  # until this many reports in a row have lost next to nothing: the link has changed.
# Far under aiortc's own floor (500 kbit/s), for a capped mobile link: a soft picture is still a picture. It starts low:
# a good link climbs to its rate in seconds, while a bad one would lose the first pictures, which every later one needs.
MINIMUM_BPS, START_BPS, MAXIMUM_BPS = 120_000, 300_000, 1_500_000
# Resends get a budget. A browser asks again for each lost packet until it gives up, and aiortc answers every request: at
# a 300 kbit/s cap that reached 600 packets a second (measured), so the link stayed full of old packets and no new picture
# got through. But where loss is the radio's and not a cap, resends are what make a picture at all: 30 % lost needs about
# 15 packets a second at the lowest rate, and a budget of 3 showed nothing (measured).
RESEND_SHARE = 0.5
RESEND_MINIMUM_PER_S = 15.0
RESEND_PACKET_BYTES = 1200
RESEND_BURST = 4  # Packets: a short gap is filled at once, at any rate.
RESEND_AGAIN_S = 0.15  # The same packet again only after about a round trip: an answer may be on its way.
KEYFRAME_EVERY_S = 2.0  # Forced whole pictures: a browser repeats its request until one arrives, and each is big.


class ResendBudget:
    """Which requests to resend a packet to answer, at `rate`'s target. `clock`: seconds."""

    def __init__(self, rate, clock):
        self.rate, self.clock = rate, clock
        self.tokens, self.at = float(RESEND_BURST), clock()
        self.sent = {}  # When each packet was last resent.
        self.resent = self.refused = 0

    def allow(self, sequence):
        now = self.clock()
        per_second = max(RESEND_MINIMUM_PER_S, RESEND_SHARE * self.rate.target / 8 / RESEND_PACKET_BYTES)
        self.tokens = min(RESEND_BURST, self.tokens + (now - self.at) * per_second)
        self.at = now
        if self.tokens < 1 or now - self.sent.get(sequence, -RESEND_AGAIN_S) < RESEND_AGAIN_S:
            self.refused += 1
            return False
        self.tokens -= 1
        self.resent += 1
        self.sent[sequence] = now
        if len(self.sent) > 512:
            self.sent = {key: at for key, at in self.sent.items() if now - at < RESEND_AGAIN_S}
        return True


class RateControl:
    def __init__(self, start=START_BPS, minimum=MINIMUM_BPS, maximum=MAXIMUM_BPS):
        if not 0 < minimum <= start <= maximum:
            raise ValueError('A rate starts between its minimum and maximum')
        self.minimum, self.maximum = minimum, maximum
        self.allowed, self.estimated = float(start), None
        self.ceiling, self.calm = None, 0

    def estimate(self, bps):
        """The receiver's estimate of what the link carries (its REMB)."""
        self.estimated = float(bps)

    def loss(self, fraction):
        """A receiver report's fraction lost, 0 to 1."""
        if not 0 <= fraction <= 1:
            raise ValueError('A fraction lost is 0 to 1')
        if fraction > LOSS_DOWN:
            self.ceiling, self.calm = self.allowed, 0
            self.allowed *= 1 - 0.5 * fraction
        elif fraction < LOSS_UP:
            self.calm += 1
            if self.calm >= FORGET_REPORTS:
                self.ceiling = None
            near = self.ceiling is not None and self.allowed >= NEAR_CEILING * self.ceiling
            self.allowed *= CAUTIOUS_GROWTH if near else GROWTH
        self.allowed = max(self.minimum, min(self.allowed, self.maximum))

    @property
    def target(self):
        """The rate to encode at now, in bits per second."""
        allowed = self.allowed if self.estimated is None else min(self.allowed, self.estimated)
        return int(max(self.minimum, min(allowed, self.maximum)))
