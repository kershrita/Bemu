# Version 1.5 — Purchase list (draft for approval)

**Status:** draft for the owner's decision at Purchase gate 1. Nothing has been bought.
**Prepared:** 2026-11-19, from web searches made that day. **Every price below is a listing
snapshot, not a quote.** Open the live page before paying; shops change price and stock daily.

How to read the "Evidence" column: **listed** = a price seen in a search result on the date above
(the page was not opened and shipping to Saudi Arabia was not confirmed); **reference** = a price
in another country, useful only for scale; **not priced** = no number was found, so none is
guessed. Totals add only the items marked *listed*.

Exchange rate used for scale only: 1 USD = 3.75 SAR (the riyal is pegged).

## 1. What the first purchase is

Roadmap stage 1 (version 2, one real leg on the bench) plus the print orders it needs. Buying the
whole robot at once is not recommended: the first leg tests the torque, the fit and the supplier
quality before 16 more servos are paid for.

### 1.1 Order of events

1. **Order the tolerance coupon alone** (`cad/export/tolerance_coupon.stl`, about 60 g of PETG).
   It is cheap and quick. When it arrives, measure the pockets and holes with a caliper and
   choose the allowance (`docs/08-mechanical-design.md`, "Tolerance coupon").
2. **Order the servos and the bench items** (table 2) at the same time, so the first servo can be
   measured (case holes, horn height) while the coupon prints.
3. **Order the first leg's parts** once the measured values are entered in `config/components.yaml`
   and `python3 tools/generate_cad.py` reports "READY TO ORDER" (not "NOT READY TO ORDER").

### 1.2 Servos and bench items (stage 1)

| # | Item | Qty | Exactly what to buy | Unit price | Evidence | Where |
|---|---|---|---|---|---|---|
| 1 | Feetech STS3215 servo, 12 V, 30 kg·cm, 1:345 | 4 (3 + 1 spare) | **12 V** version, datasheet ST-3215-C047, sold as **C018**. **Not C001 (7.4 V).** Ask for the accessory bag (horns and screws). | $22–39 | listed: Comet (B2B) $22 excl. VAT; Robotshop UK £19.20; aifitlab $26 (backordered); rcdrone.top $29 (sold out); eBay US stock $38.94 | Official Feetech store or authorised reseller on AliExpress (not checked: no AliExpress page was reachable); the shops above; Waveshare |
| 2 | Waveshare Bus Servo Adapter (A), USB | 1 | Waveshare "Bus Servo Adapter (A)" for ST/SC series; also lets the PC talk to the servos | — | not priced (listings found, no number kept) | waveshare.com, official resellers |
| 3 | 12 V bench power supply, current-limited, 10 A or more | 1 | Adjustable 0–30 V / 10 A lab supply | — | not priced (docs/10 guesses $25–40, unverified) | local electronics shop, Amazon.sa, Noon |
| 4 | Pi 5 active cooler, 27 W supply, microSD | — | Only if not already owned | — | skip if owned | — |
| 5 | Heat-set inserts, M3 × 4 mm (brass) | 1 pack of 50 | **Length 4 mm** (the top plates are 4 mm thick); outer diameter near 4.6–4.9 mm; hole in CAD is 4.1 mm, confirm with the coupon | — | reference only: Grobotronics assortment 7.92 (currency not shown) | AliExpress, Amazon.sa |
| 6 | Screws: M2 (servo mounting, length per `print_order.md`), M3 socket-head assortment | 1 set | Stainless socket-head, M2 × 6–10 and M3 assortment | — | not priced (docs/10 guesses $10–15, unverified) | AliExpress, Amazon.sa |
| 7 | Multimeter, soldering iron | 1 each | Only if not already owned (needed to fit the inserts) | — | not priced (docs/10 guesses $15–25 each, unverified) | local |

**Priced total for item 1 alone (4 servos):** about **$88–156** at $22–39 each. The low end is a
business price excluding VAT, and the two $26–29 listings (aifitlab, rcdrone.top) were backordered or
sold out when seen; the only in-stock listing seen was $38.94 (eBay, US stock). **Plan on about
$156 for four**, and treat anything lower as a bonus that depends on stock. Shipping and any
import charges are extra.

## 2. The full robot (stage 2, bought only after version 2 passes)

| # | Item | Qty | What to buy | Rough price (a guess unless Evidence says *listed*) | Evidence |
|---|---|---|---|---|---|
| 8 | Servos, same model | 16 (15 + 1 spare) | As item 1; with item 1 this makes 20: 18 in the robot, 2 spares | $22–39 each, **$352–624** | listed (as item 1) |
| 9 | Battery, 3S LiPo, 5000 mAh, 40–50C, XT60 | 1 | e.g. Ovonic or GENS ACE 3S 5000 mAh. Ovonic's 5000 mAh pack is listed at **359 g, 138 × 42 × 28 mm** (the body's placeholder is 350 g; whether it fits has not been checked, because the battery bay is not designed yet). | $45–85 | reference: GENS ACE 3S 5000 mAh 50C AED 305 excl. VAT (about $83) at a UAE shop; Ovonic 5000 mAh not priced |
| 10 | Balance charger for 3S LiPo, and a fire-resistant charging bag | 1 + 1 | Any reputable 3S charger with a balance port | $30–60 | not priced (rough estimate from docs/10, unverified) |
| 11 | Step-down converter, 12 V to 5.1 V, 5 A or more, with a **USB-C connector that identifies itself to the Pi 5 as a 5 A supply** | 1 | See the caution below | $8–20 | not priced (no listing price found; 27 W wall adapters are $13–18, which is not what is needed; $8–20 is a guess) |
| 12 | Main switch, fuse holder, 20 A fuses, silicone wire (14/16/20 AWG), XT60 connectors, bulk capacitor (1000 µF, 25 V) | 1 set | | $20–35 | not priced (rough estimate from docs/10, unverified) |
| 13 | Servo bus cables and a distribution board | as needed | Use the cables supplied with the servos; hand-wire a junction | $5–15 | not priced (rough estimate from docs/10, unverified) |
| 14 | TPU feet (printed) or rubber tips | 6 | Printed in TPU by the service, or bought | $5–15 | not priced (rough estimate from docs/10, unverified) |
| 15 | Low-voltage cell alarm | 1 | Independent backup | $3–5 | not priced (rough estimate from docs/10, unverified) |

**Caution (item 11).** The Pi 5 only draws its full 5 A when its USB-C port negotiates a 5 A
(27 W) supply. A plain 5 V buck converter without that negotiation makes the Pi limit its USB
current and raise a power warning; use a module sold specifically for the Pi 5 or power the Pi
from the 5 V header per the Pi documentation, and test it on the bench before it goes in the
robot. Do not buy until `docs/09-electronics-and-power.md` has been re-read.

## 3. Printing (not priced: a quote is needed)

Nothing in the printing market of Saudi Arabia was found with a published per-gram price. The one
local listing found (a Haraj classified, individual seller, "from 5 riyals", price by sending the
file on WhatsApp) is not a verified company and does not mention PETG. So printing is **quote
based**: send the same request to two or three services and compare. The request, ready to send in
English and Arabic, is `docs/reports/v1_5-print-quote-request.md`.

Material for the robot (computed from the CAD, 60% fill estimate, `cad/export/printability.md`):
**PETG about 664 g, TPU about 7 g**; about 764 g of PETG with a 15% allowance for supports, purge
and one failed print. One 1 kg spool is enough. Largest footprint is 190 mm, so the parts fit a
200 × 200 mm bed.

## 4. Getting it into Saudi Arabia

These notes come from search results on 2026-11-19 and **were not confirmed with the customs
authority**. Check the official pages before relying on any figure.

- **Duty:** ZATCA's page for individual e-store purchases says goods are exempt from customs duty
  when the **total (goods + shipping + insurance) is under 1,000 SAR** and no clearance permit is
  needed. Split orders accordingly, but do not expect the exemption on a single large order.
- **VAT:** sources disagree. One carrier page says 5% VAT on all non-document express shipments;
  others say 15%. Budget 15% on top of the price until you see a real invoice.
- **Handling:** couriers may add their own customs handling fee (roughly SAR 20–100 per parcel,
  carrier dependent).
- **Lithium batteries:** shipping and customs rules are stricter (declaration, labelling, SASO
  conformity). **Buy the battery inside Saudi Arabia** from a shop that sells RC LiPo packs.
- **Counterfeits and wrong variants:** buy servos from the official Feetech store or an authorised
  reseller, and check the label: **12 V / 30 kg·cm / C018**. A 7.4 V C001 listing is the wrong
  servo for this design.

## 5. On arrival: what to check (do this before printing more)

1. Servo label says 12 V and the model matches.
2. Caliper the first servo against `docs/08-mechanical-design.md` "What to measure" and write the
   values into `config/components.yaml` (case hole thread, case shelf positions, horn disc and
   height, rear hub and idler positions).
3. Try one of the accessory screws in the bracket hole and record its thread.
4. Power one servo from the bench supply through the adapter and confirm it moves and reports its
   position, voltage and temperature.

## 6. Decisions needed from the owner

- [ ] Approve the first purchase (items 1, 2, 3, 5, 6) and the coupon print.
- [ ] Choose which print service to quote first (one in Saudi Arabia and one overseas, so the
      prices can be compared).
- [ ] Say whether to buy all 20 servos now or 4 first (recommended: 4).
