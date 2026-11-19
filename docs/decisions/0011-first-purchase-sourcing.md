# 0011 — Where the first purchase comes from

**Status:** Proposed (the owner approves it at Purchase gate 1)
**Date:** 2026-11-19
**Decided by:** Designer, proposed to the owner

## Context

The first purchase is one leg's worth of servos, the bench tools and the first print orders. The
owner lives in Saudi Arabia and wants good price and quality, from China or anywhere. Counterfeit
and wrong-voltage servos are common, shipping lithium batteries is restricted, and customs rules
change. No Saudi printing service with a published price was found.

## Decision

1. **Servos:** the 12 V, 30 kg·cm Feetech STS3215 (C018), from the official Feetech store or an
   authorised reseller, four at first (three for the leg and one spare). The other 16 wait for
   version 2 to pass.
2. **Battery, charger, bags and fuses:** bought inside Saudi Arabia, not imported.
3. **Printing:** quote-based. Send the same request to at least one Saudi service and one overseas
   service; start with the tolerance coupon alone. Parts are designed for a 200 × 200 mm bed.
4. **Order size:** keep each import parcel under 1,000 SAR in total where practical, to stay
   inside the customs-duty exemption (to be confirmed with the courier).
5. **Nothing is bought until the owner approves the list in
   `docs/reports/v1_5-purchase-list.md`.**

## Alternatives considered

| Alternative | Strength | Why not |
|---|---|---|
| Buy all 20 servos at once | One shipment, possible volume discount | $350–620 at risk before one leg works |
| Dynamixel servos | Best documentation, official ROS 2 driver | The affordable model fails the torque rule (docs/10) |
| Import the battery | Possibly cheaper | Lithium shipping rules and customs documentation; local purchase avoids both |
| Own printer | Fast iteration | Not chosen by the owner (decision 0009) |

## Consequences

- The first leg tests supplier quality before the bulk of the money is spent.
- Prices in the purchase list are listing snapshots, not quotes, and must be rechecked.
- Printing cost enters the budget only once a quote exists.

## References

The searches behind the numbers are listed in the purchase list; the project's references are in
[References](../references.md).
