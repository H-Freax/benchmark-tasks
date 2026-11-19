# Privacy-screen engineering assembly and reset proposal

This document describes one original, manufacturable engineering equivalent for the existing three-panel screen. It is not OEM CAD, a measured specimen, a safety certification or a claim that the physical assembly has been built. All native dimensions are transcribed from the frozen geometry. The proposed hinge, fastener and foot-mount dimensions are author-selected fabrication dimensions. Unmeasured fits, adhesive selection, hinge resistance, friction and actual mass remain explicitly open. Ordinary ideal hinge and foot approximations may be disclosed; the final reviewer must judge their material effect. A percentage of omitted hardware mass alone is not an official rejection rule or a requirement to make another native version.

The drawings specify an unbuilt assembly proposal; native dynamics and physical validation are separate evidence.

## Drawing index and authority

- `01-panel-elevation.svg`: exact560mm nominal width,1652mm height and300/1300mm proposed hinge elevations.
- `02-hinge-and-foot.svg`: proposed pin, sleeve, leaf, spacer, mounting holes and bolted foot section.
- `03-u-reset-template.svg`: exact native U-shaped starting layout, six feet, axes, robot pedestals and measurement marks. It is a coordinate drawing, not a1:1printout; use the listed millimetres or create a calibrated1:1floor template.
- `04-contact-directions.svg`: useful and blocked folding directions, with finite-solid contact examples.
- `ASSEMBLY-SPEC.json`: exact per-body hole coordinates, original component list and volume-based hardware estimate. It is the numeric authority when the screen illustration is schematic.

Blue shapes depict the native source; orange hardware is the new physical proposal. The material class follows the [AGA medical-screen product description](https://www.agasan.com/en/products/medical-screens/):30x20x2mm aluminium framing and3mm rigid PVC foam,1652mm height. That reference does not supply these560mm-panel cut lengths, the hinge offsets, pin/leaf dimensions or this assembly's mass. The advertised1690mm extended width,605x82mm folded size and11.6kg are OEM properties and are not inherited by this original equivalent.

## Panel and rigid construction

Each panel has two30x20x2mm aluminium vertical tubes1622mm long, at localx0..30 and530..560, z30..1652. Two500x20x30mm horizontal tubes are fitted between them atz50..80 and1622..1652. The frame's nominal outside width is560mm. The500x3x1542mm PVC foam inlay occupiesx30..530, y-1.5..1.5, z80..1622. Each rubber outer foot envelope is32x24x30mm, centred atx15 or545,y0,z15; the source therefore reaches localx-1..561, a maximum562mm width including feet.

Fixture the tubes square to these dimensions and butt-weld the four corners. Use an aluminium-compatible welding process/filler selected for the actual alloy; the source does not specify a certified alloy/temper or weld procedure. Hold the inlay in the source aperture with compatible structural adhesive fillets of proposed0.5mm radius along both faces of its perimeter, rather than assuming an unattached board remains fixed. Adhesive product, surface preparation, strength and cure require selection and verification. Small weld/adhesive volumes are omitted by the native solid model and are not included in its mass. These details are not structural load certification.

The current source mass is3.3156072kg per panel and9.9468216kg total. It integrates tube walls, a solid PVC inlay and solid rubber blocks at declared densities. It includes no hinge hardware, fasteners, weld filler, adhesive or foot inserts. Do not label it a measured physical mass.

## Hinge axis and two-bearing implementation

The mathematical axis has no required physical pin atz0. The following pair of short, coaxial hinges at localz300 and1300mm represents each vertical revolute connection. The two bearings constrain the same relative rotation; neither is a ground attachment.

| Joint | Centre-panel axisx,y | Wing axisx,y | Initial world axisx,y |
|---|---|---|---|
| Left | 0,-15 | 0,-15 | 965,-280 |
| Right | 560,+15 | 0,+15 | 935,+280 |

All numbers are millimetres. In the settled upright physical assembly the bearing centre heights are300 and1300mm above the foot contact plane. The source's reference anchorz0 is a point on that same infinite axis. Use a single straight alignment jig through both bearings during assembly, then remove it. Proposed coaxiality and hole-location tolerances are0.2mm; they are original fabrication targets, not measured achievable values or native solver settings.

Each of the four proposed short hinges has two60mm-high,28mm-reach,1.5mm-thick304 stainless leaves, two1mm aluminium face spacers and five interleaved stainless knuckle sleeves. Each sleeve isOD8/ID4.2mm and11.6mm long. Four0.5mm axial gaps make a60mm bearing stack. Knuckle centres relative to the bearing centre are-24.2,-12.1,0,+12.1,+24.2mm; owners alternate centre/wing/centre/wing/centre. Weld the sleeves only to their own leaf, preserving the other leaf's gaps. The pin is a turned4mm-diameter60mm smooth shoulder,7x2mm head andM3x5mm lower retention end, with washer and locking nut. Retain without axially clamping the rotating stack. No spring, latch, motor or friction brake is added by this proposal. Bore/pin clearance and straightness need bench verification; no measured hinge torque is supplied.

For a negative-y face, the rail surface isy=-10mm, the spacer occupies-11..-10 and the leaf-12.5..-11, while the sleeve axis isy=-15. Mirror those thickness coordinates for the positive-y face. The right centre leaf extends inward fromx560 tox532; the other three leaf mount families extendx0..28. This is an offset surface leaf, not a zero-thickness connection through the rubber foot.

Mount each leaf with twoM4x30mm through-bolts, two washers per bolt andM4 locking nuts. Drill4.5mm through the two opposite tube walls and spacer/leaf. Holex is16mm from the first rail edge, orx544 on the centre panel's far rail; holez is hinge centre±18mm. The four holes on each mounting rail are therefore atz282,318,1282,1318mm. Exact body-localx/y/z hole centres for both joint sides are listed in JSON. Support the hollow tube while tightening; prevent crushing the2mm walls. Actual bolt head/nut dimensions, protrusion, grip length and torque must be checked for the selected fasteners. Do not substitute an assumed friction setting for a measured resistance curve; the native0.08Nms/rad viscous damping is an explicitly ideal approximation.

## Rubber foot attachment

The native foot is a solid32x24x30mm rubber equivalent, tangent to the tube end atz30. A normal sleeve cap that extends over the tube would not be this exact solid volume. The proposed physical attachment instead uses the same outer rubber block, a recessed axial screw and an internal end plug:

1. Fit a25.6x15.6x20mm acetal plug into the nominal26x16mm tube cavity atz30..50. Check the real tube's corner radii; machine the plug to clear them. The nominal0.2mm radial fit allowance is a proposed starting fit.
2. Retain the plug using twoM3x25mm cross fasteners at localz40 andx8/22mm, measured from that vertical rail's first edge. For the far rail add530mm tox. Their3.2mm bores and nuts must be checked against the actual fastener stack.
3. Drill a4.5mm axial bore in the rubber foot, centred on the rail. Recess the underside8mm diameter by4mm. Use anM4x40 screw into a15mm-deep axialM4 plug thread.
4. Select a head that remains at least1mm above the foot contact plane after compression; no metal may touch the floor. Confirm pull/shear retention before dragging the panel.

The native foot omits the bore, plug and screws and treats rubber as rigid. This is a documented approximation. The actual foot material/compression and floor friction must be checked; the native rubber static/dynamic coefficients0.45/0.36 are engineering assumptions, not validated product data. A selected ordinary commercial foot may be used instead if its geometry and effect are documented.

## U-shaped reset

This starts partly folded in a U, not in a compact accordion pack. In the native world frame, origins are centre(950,-280) at90degrees, left(965,-295) at180degrees and right(935,295) at180degrees. The initial screen bounds including its native feet arex374..966,y-307..307, hence592x614mm in plan and1652mm above the foot plane.

| Panel | First foot centrex,y | Second foot centrex,y |
|---|---|---|
| Centre | 950,-265 | 950,+265 |
| Left | 950,-295 | 420,-295 |
| Right | 920,+295 | 390,+295 |

Mark these six rectangles and headings on a level floor. Place the independent140x140mm pedestal footprints at(150,+550) and(150,-550); arm mounts are750mm above their bottoms. Mark the target front facex1496, y-520..520. Its189dot centres arex1495,y-500..500 every50 andz600..1500 every112.5mm. The three photo centres are(0,-300,1100),(0,0,1100),(0,+300,1100). Target/pedestal world coordinates remain as in the task setup.

Native authored panel roots start atz0 before gravity settling; the station ground top isz=-12.5mm, so settled screen roots move down12.5mm. For a real floor, use the physical floor as the foot-plane datum, record its offset to the world measurement coordinates and keep target/photo heights consistent. Do not place the target at an unexplained1652mm-relative datum. Remove human support and all temporary alignment/reset jigs before a trial. Verify all feet touch and both hinge axes remain intact. Restore the same U layout, not the previous successful endpoint.

## Contact directions and disclosed implementation differences

Adjacent panels have native collision enabled. Do not force a wing through the centre panel or through the other wing. Mathematical revolute limits alone do not guarantee clearance of finite tubes, inlays, feet or proposed hinge hardware. The contact drawing illustrates useful and blocked travel directions; those example angles are not score gates. Check the full physical hinge/fastener sweep before a real trial.

The specified nominal leaves, knuckles, spacers and smooth pin sections have a volume-based estimate of about280.85g total,2.82percent of the native screen mass. This excludes pin heads/threads, bolts, washers, nuts, foot inserts, welding and adhesive; it is neither a weighed mass nor a universal minimum for every hinge design. This particularOD8 sleeve reaches initialworldx969,3mm beyond the native maximumx966, making this proposal's complete hardware width at least595mm. This does not prove that every physical hinge implementation must differ: a thinner commercial hinge or inset mounting is an alternative, but is not selected or dimensioned here. No82mm folded-envelope claim is made.

For this selected physical implementation, review whether the added protrusions or inertia affect robot grasp, folded clearance, contact, floor sliding, tipping or privacy coverage. If material, the smallest matched-source revision is to add its actual hinge/fastener/foot envelopes and mass/COM/inertia to the existing rigid panels while retaining the original two axes, stock robot/controller limits and functional scoring; then use matched run evidence. If judged negligible at the tested configurations, disclose and justify the idealization. This documentation does not silently apply that change or force a new runtime version. Publication equivalence and remaining validation are separate decisions.
