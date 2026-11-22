# BUILD TONIGHT: 5in lead, Ender 3 S1 / PETG structural spin test (plain ESC, no cyclic)
Generator `../build_5in_ender_tonight.py [t_lam_mm] [n_lam]` (re-run with the measured can thickness). CAD = SolidCraft script `rotor_script.json` (replays in Keel). Flexure geometry is the lead's (free length 10.05 mm, skew 62 deg, cross 105 deg, hinge radius 10.75 mm); only the strip stack, the slots and the boss change.

## Parts
- `hub.stl` x1, `blade_a.stl` x1, `blade_b.stl` x1 (blade_b is the z-mirrored fitting of blade_a with the same airfoil, so it is NOT the same part; print both).
- Strips: drink-can aluminium, 3 per stack x 0.10 mm (measure the can wall: re-run with the real t), 4.0 mm wide, 26.05 mm long (`strip_template.svg`, 1:1; check the 10 mm bar first). 4 stacks = 12 strips; cut 16.
- Slots: 0.55 x 4.2 mm (n t + 0.25 by 4.0 + 0.2), 8.0 mm glue land each side, 0.5 mm 45 deg mouth chamfer. The hub slot goes through (load the stack from the hub's back). The blade slot is blind and exactly one land deep, so the stack bottoms out and sets the 10.05 mm free length.
- Boss: OD 10 x 6.5 mm, bore 5.1 through. The bottom face is the lead's (z -6), the top is at z +0.5, and the washer/nut seat above it is cleared to 11 mm dia. The upper (blade_b side) arms meet the boss through the spine. Aramid strand: optional for a spin test. The channels and wells are in the parts.

## Print (PETG, Ender 3 S1)
- 0.2 mm layers, 4 perimeters, 40 % gyroid, 240/80 C, 0.4 nozzle. STLs are in the rotor frame (z = rotor axis, up).
- WARNING (geometry, not a slicer issue): with the lead's skewed crossed strips, the hub jaws are 33 deg inclined slabs that hang about 4.8 mm below the boss bottom face, and the blade jaws rise above the fitting plate. No orientation of hub or blade gives a flat face down without supports. Jaw faces are about 45 deg, so they print, but they need something to start on: use tree supports from the build plate for the boss and arms (hub), and under the blade root fitting and the twisted blade root (blades). The flat-bottomed hub asked for needs a flexure redesign; that is not done here.
- WARNING (motor): the hub's lower jaws sit within about r 15 mm and 4.8 mm below the boss face, so they will hit a 2307 bell. Put a spacer of 6 mm or more (stacked M5 washers or a printed tube) between the bell and the hub, then check the free spin by hand before powering up.

## Assembly
1. Clean the slots with the strip template blank: a dry 3-strip stack must slide in through the mouth chamfer. Do not force it. Sand the stack, not the slot.
2. Cut strips square with a sharp blade on glass, deburr, then wipe with IPA. Stack 3 DRY: no glue between laminae, as they must slide in the free span.
3. Dry fit: blade on the bench, hub beside it. Push each stack from the hub's back through the hub slot, across the gap, into the blade slot until it bottoms out. Check the 10 mm free span with calipers, then the cross-over (the two stacks must not touch).
4. Glue: put a bead of thick epoxy or CA GEL on the slot LANDS only, at the outer mouths. Keep it off the free span and the jaw exits, and use no thin CA. Wipe any creep at once. Cure fully (epoxy 24 h; CA gel about 1 h plus kicker only at the mouths).
5. Order: blade_a's 2 stacks, then blade_b's 2 stacks, both with the hub in the same jig pose. Then the spacer, hub, M5 washer, locking nut.
6. Static check before spinning: hang 10 g at 50 mm radius on each blade (hub clamped). The expected deflection is about 2.9 deg (the main session's estimate, not computed in this CAD). Much stiffer means glue got into the free span. Much softer, or uneven between the two stacks, means a lamina slipped: redo that stack.
7. Pin-balance the blade pair, then spin up behind a shield, slowly, with nobody in the plane of the rotor.
