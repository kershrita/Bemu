# Print quote request (send to two or three services)

Send the **English or Arabic text below**, with the files listed, to each service. Compare the
quotes on: price for the same material, delivery time, build-bed size, and whether they print
TPU. Start with order A only; order B follows once the coupon has been measured.

The files are in `cad/export/` (`.stl` for printing; `.step` if the service prefers it).
Quantities and orientation for every file are in `cad/export/print_order.md`; sizes and support
estimates are in `cad/export/printability.md`.

| Order | Files | When |
|---|---|---|
| **A: tolerance coupon** | `tolerance_coupon.stl` × 1 | now, alone |
| **B: one leg** | the 9 leg files in `print_order.md` at the quantities for **one leg**, plus 1 spare coxa bracket | after the coupon is measured |
| **C: body and five more legs** | body plates (3 + 3 pieces), hip plates, posts, splice plates, and the leg files × 5 | after version 2 passes |

## English

> Hello, I would like a quote for FDM (filament) 3D printing, delivered to <city>, Saudi Arabia.
>
> - **Material:** PETG (ABS, ASA or nylon is acceptable if PETG is not available). One small part
>   family, the robot feet, in TPU (flexible, shore 95A) if you offer it; otherwise skip them.
> - **Settings:** 0.2 mm layers, 4 perimeters (about 1.6 mm walls), 35% infill, any colour.
>   Orientation per file is given in the attached `print_order.md` (plates flat, posts upright).
>   Small supports are needed on a few parts only (listed in `printability.md`).
> - **Size:** the largest part is 190 × 80 × 4 mm; the tallest is 50 mm. A build volume of
>   200 × 200 × 80 mm is enough.
> - **Accuracy matters:** holes and pockets must come out at the modelled size. Please do not
>   enable "hole compensation" or scaling. I am ordering a small test coupon first so I can
>   measure your printer's accuracy before ordering the real parts.
> - **Files:** attached (STL). Quantities per file are in `print_order.md`.
>
> Please tell me: the price per order (A, then B), the delivery time, what layer height and
> nozzle you use, and whether you can print TPU. Thank you.

## العربية

> السلام عليكم، أرغب في عرض سعر لطباعة ثلاثية الأبعاد بتقنية FDM (الخيط)، مع التوصيل إلى <المدينة>،
> المملكة العربية السعودية.
>
> - **الخامة:** PETG (ويمكن استخدام ABS أو ASA أو النايلون إذا لم يتوفر PETG). قطع صغيرة
>   (أقدام الروبوت) بخامة TPU المرنة (شور 95A) إن كانت متوفرة لديكم، وإلا فتُستثنى.
> - **الإعدادات:** سماكة الطبقة 0.2 مم، 4 جدران (حوالي 1.6 مم)، تعبئة 35%، أي لون.
>   اتجاه الطباعة لكل ملف موضح في ملف `print_order.md` المرفق (الألواح مسطحة والأعمدة عمودية).
>   بعض القطع تحتاج دعامات صغيرة فقط (موضحة في `printability.md`).
> - **المقاس:** أكبر قطعة 190 × 80 × 4 مم، وأطولها 50 مم. يكفي حجم طباعة 200 × 200 × 80 مم.
> - **الدقة مهمة:** يجب أن تخرج الثقوب والتجاويف بالمقاس المصمم. أرجو عدم تفعيل تعويض الثقوب
>   أو تغيير المقياس. سأطلب أولاً قطعة اختبار صغيرة لقياس دقة الطابعة قبل طلب القطع الفعلية.
> - **الملفات:** مرفقة بصيغة STL. الكميات لكل ملف موجودة في `print_order.md`.
>
> أرجو إفادتي بالسعر لكل طلب (أ ثم ب)، ومدة التسليم، وسماكة الطبقة وقطر الفوهة المستخدمين،
> وهل تتوفر الطباعة بخامة TPU. شكراً لكم.

## After the quotes arrive

Record the chosen service, its build size, material and price in `docs/reports/v1_5-purchase-list.md`
(section 3) and in `docs/15-risks-and-open-questions.md` (question Q5, which is closed by this).
