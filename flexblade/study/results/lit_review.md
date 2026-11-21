# Literature review: torque-modulated flexure cyclic rotor (2026-11-21)

Scope: which of our simulation results are already known and which look new. "Read" means I read
the full text (pdftotext). "Abstract" means I saw only the abstract or a summary page. "Memory" means a
classic reference cited from memory and not re-checked online. Verdicts: KNOWN / PARTLY KNOWN / LIKELY NEW.

## Core prior work (the Penn line and its descendants)
- Paulos & Yim, "An underactuated propeller for attitude control in MAVs", IROS 2013. Skewed pin lag hinges,
  +/- lag-pitch coupling, no flap hinge. https://www.modlabupenn.org/wp-content/uploads/paulos_an_underactuated_propeller_IROS_2013.pdf (abstract)
- Paulos & Yim, "Flight performance of a swashplateless MAV", ICRA 2015. A 227 g coaxial vehicle with a 30 cm rotor,
  a hall/magnetic encoder on a custom ESC, and a calibrated phase offset. https://www.modlabupenn.org/wp-content/uploads/paulos_flight_performance_ICRA_2015.pdf (read)
- Paulos & Yim, "Cyclic blade pitch control without a swashplate for small helicopters", JGCD 41(3) 2018,
  doi:10.2514/1.G002683. Flap hinge plus skewed lag hinge, 32 cm rotor, a 4096-count encoder, a 2 kHz PI speed governor, and a full model.
  https://www.modlabupenn.org/wp-content/uploads/2018/05/paulos_cyclic_blade_pitch_JGCD_2018.pdf (read)
- Paulos & Yim, "Scalability of cyclic control without blade pitch actuators", AIAA SciTech 2018,
  doi:10.2514/6.2018-0532. Tested 10 cm and 1 m rotors and gives the scaling laws. https://www.modlabupenn.org/wp-content/uploads/2018/03/paulos_scalability_of_cyclic_AIAA_2018.pdf (read)
- Paulos PhD thesis, "Rotorcraft blade pitch control through torque modulation", UPenn 2018. https://repository.upenn.edu/edissertations/2901 (abstract)
- Patents: US9914535 (Paulos/Yim, UPenn; it includes **living-hinge flexures replacing pin hinges**, a "monolithic
  mass-producible flexible propeller"); US11345469 and US11834164 (Piccoli / IQinetics = Vertiq; "torque applied to a
  flexible rotor", damping in "hinge or flexure"); Fujihira et al. US5259729 (1993; a torsional cantilever spring
  between the motor and the prop twists the blade under torque pulses); Reich WO2005087587 (2005; a hinged or flexible blade
  responding to torque). https://patents.google.com/patent/US9914535B2 , https://patents.google.com/patent/US11345469B2
- Vertiq "Pulsing" firmware docs: a commercial product. Velocity-mode pulsing adds a sine to the speed-loop target
  and works only with flapping/teetering Vertiq props. https://iqmotion.readthedocs.io/en/latest/manual/manual_pulsing.html (read)
- Chen, ..., Lyu, F. Zhang (HKU MARS), "Swashplateless-elevon actuation for a dual-rotor tail-sitter", arXiv 2309.13559 (2023).
  A commercial T-Motor ESC driven over DShot600 open-loop throttle, an AS5600 encoder at 910 Hz read on the FC, and a fixed
  phase-delay constant gamma0 "calibrated in advance" on a test stand. Bearings were added to cut hinge friction. (read)
- Liang, Guo, Lyu, arXiv 2511.04251 (2025). A coaxial tailsitter; flap hinges were added "to reduce vibration during cyclic
  acceleration/deceleration". (abstract)
- Leines Artieda, Stewart, Benedict (Texas A&M), VFS Forum 81, 2025, doi:10.4050/F-0081-2025-0067. A single 45 deg tilted
  lag hinge plus flap hinges, with RCAS simulation and wind-tunnel validation. Peak lag and pitch are **180 deg behind peak torque**. (abstract)
- Bhatnagar et al., VFS Forum 80, 2024, "Swashplateless micro helicopter with pitch-lag coupling" (a Vertiq UP12 drive). (abstract)
- Tang et al., Int. J. Micro Air Vehicles 2024, "Parameter effect on the novel swashplateless rotor control",
  doi:10.1177/17568293241238019. They note that flap-hinge-less designs give blade-to-blade phase asymmetry and instability. (snippet only; paywalled)
- J. Fluids & Structures 2025, "Numerical simulations of passive swashplateless rotors" (S0889974625001264). It reports that
  hinge friction causes the phase delay in control moments. (snippet only; paywalled)
- Hobby/maker: J. Bird single-propeller drone (2026; github.com/jyjblrd/single-propeller-drone; 6" prop at ~20k rpm;
  a stock sensorless ESC "lost sync above ~50 pulses/s", so he switched to moteus plus an encoder; a 160 Hz blade-bending
  vibration was fixed by stiffer blades). "Worked in theory" printed micro rotor (2026; hinge friction dead zone below
  3000 rpm; overshoot/resonance cured by tip masses). DroneXL write-ups (secondary sources):
  https://dronexl.co/2026/10/20/joshua-bird-single-motor-drone-hinged-propeller/ ,
  https://dronexl.co/2026/10/21/worked-in-theory-3d-printed-micro-helicopter-rotor/

## Claim-by-claim

**1. Underactuated torque-modulated rotor with passive skewed hinges: KNOWN.**
Paulos & Yim 2013/2015/2018 cover the concept, the opposite-sign lag-pitch coupling, phase locking, models and
flight. HKU (2023-25), Texas A&M (2025), Vertiq (commercial) and several hobby builds have repeated it. Our contribution
is not the principle.

**2. Bearingless / flexure-hinge version: PARTLY KNOWN.**
Flexures are explicitly claimed as an embodiment in US9914535 (living hinges) and US11345469 ("flexible rotor",
"hinge or flexure"). Fujihira 1993 used a torsion-spring flexure. Full-scale bearingless rotors (flexbeams, BMR, Bo-105
hingeless) routinely build in pitch-lag/pitch-flap coupling by flexbeam geometry (memory: Johnson, *Helicopter Theory* 1980;
Ormiston & Hodges 1972 hingeless flap-lag stability). I found **no published design, model or test of a crossed-strip
(cross-spring pivot) skewed flexure used as the lag-pitch coupler**, and nothing on steel-shim plus aramid/CF strap hubs.
Specifics LIKELY NEW; the concept is covered by patents (an IP note for any commercialisation).

**3. Mirrored-flexure asymmetry, balanced with precone: PARTLY KNOWN.**
- Precone to relieve steady root flap bending on hingeless rotors: KNOWN (classic; Bo-105 has 2.5 deg built-in coning,
  Peterson & Johnson, NASA; textbook treatment in Johnson 1980 and Bramwell). Ormiston & Bousman (NASA, 1972) found **precone
  strongly destabilising** for hingeless flap-lag stability (see contradictions).
- Blade-to-blade asymmetry from the mirrored coupling: PARTLY KNOWN. Paulos JGCD 2018 (sec. II) states that without a flap
  hinge the two blades are forced to flap differently (2/rev), giving "large bending moments at the blade roots". The cure there was an
  explicit flap hinge. Tang 2024 and Liang 2025 report the same asymmetry and vibration.
- Our point is specific: a mirrored flexure is loaded from the "wrong side" by thrust/flap moment, so the two blades
  have different effective stiffness and coupling, and precone chosen to null the flap moment equalises them. I found nothing on this: LIKELY NEW.

**4. ESC speed governor vs torque-modulated cyclic; 1-rev moving-average rpm filter: PARTLY KNOWN / LIKELY NEW.**
Paulos JGCD 2018 eq. 40 folds the PI governor into the rotor dynamics as an effective hub stiffness k_m and damping c_m
(set by K_P, K_I). So "the speed loop changes cyclic gain/phase" is KNOWN in principle. They also warn that thrust bandwidth is
limited because the rotor is tuned for 1/rev and that K_P, K_I must be chosen with that in mind. Vertiq's velocity-mode pulsing
deliberately puts the pulse *inside* the speed loop. HKU used open-loop throttle with no governor. **Not found:** (a) the
governor pumping or destabilising the lag/pitch mechanism mode; (b) a per-revolution moving-average (comb, zeros at nP)
speed estimate that makes the governor blind to the 1/rev pulse and restores free-spin cyclic response. Comb or
synchronous filters for periodic disturbances are standard in motor-drive practice (generic, not rotor-specific).
Applying them here, with the mode-pumping analysis and governor-bandwidth limits: LIKELY NEW.

**5. Authority vs proximity of the mechanism mode to 1/rev; per-unit calibration: PARTLY KNOWN.**
Paulos JGCD/AIAA give amplitude/phase frequency responses with friction nonlinearity at low amplitude. TAMU 2025 shows a 180 deg lag,
meaning pin-hinge rotors operate *above* the lag-mode frequency (inertia-dominated). JFS 2025 attributes phase delay to
friction. HKU calibrates one phase constant per rotor on a test stand. Paulos AIAA 2018 notes device-to-device motor variation
and gives a noninvasive sysID. A maker build reports resonance and overshoot fixed by added tip mass. **Not found:** deliberately
placing an elastic (flexure) mechanism mode near 1/rev for resonant gain, the resulting authority/sensitivity trade, or
boot-time per-unit phase/gain self-calibration in ESC firmware. LIKELY NEW as a design study; the ingredients are known.

**6. Ground resonance / hub-tilt stiffness for small 2-blade rotors on flexible arms: PARTLY KNOWN.**
Coleman & Feingold (NACA TR 1351, 1958) and the classic 2-blade results (periodic coefficients, Floquet analysis, Coleman
transform invalid for Nb=2) are KNOWN (memory plus search snippets). Two-blade gyroscopic hub-tilt behaviour is standard rotordynamics
(2-blade wind turbines and propeller whirl). I found nothing applying it to a deliberately 1/rev-excited lag mode on drone arms.
A quantitative stiffness rule for that case: LIKELY NEW, low novelty.

**7. Fatigue of thin steel shim flexures; lateral-torsional buckling (LTB): KNOWN (methods), PARTLY KNOWN (application).**
Cross-spring pivot theory is old: Wittrick, Aeronautical Quarterly 1948/1951 (memory) and Freire Gomez, Booker & Mellor,
Precision Eng. 42 (2015) 6-21, doi:10.1016/j.precisioneng.2015.03.003 (endurance-limit design and shape optimisation; their
related thesis targets a helicopter active Gurney-flap actuator). Goodman/mean-stress fatigue for flexures is textbook (Howell,
*Compliant Mechanisms*, 2001; memory). LTB of strips under in-plane moment is classic (Timoshenko & Gere). Combining these for
centrifugally preloaded 1095 shim in a rotor hub with an aramid tension strap: LIKELY NEW as an application, but not a research result.

**8. Miniaturisation to 2-3": PARTLY KNOWN.**
Paulos AIAA 2018 already built a **10 cm (~4") rotor with a 3.1 g motor and an encoder-based custom controller** and derived
the scaling laws. The torque modulation needed scales as R^5 Omega^2, the response time as 1/Omega, and the nondimensional dynamics are scale-free apart from
the motor groups c_m/(Omega I), k_m/(Omega^2 I). Reynolds/Mach cannot both be matched. Piccolissimo (Piccoli & Yim, ICRA 2014/16;
28-39 mm) is single-motor but steers by an offset motor, not cyclic. Practical limits reported: sensorless ESC loses sync
at about 50 pulses/s (Bird); hinge friction dead zone (maker; JFS 2025); every working build uses an angle encoder. **Not found:** any flight
or test of a passive-cyclic rotor below 10 cm, or with a sensorless 32-bit ESC (e.g. AM32) at 2-3" (1/rev > 400 Hz). LIKELY NEW.

## (a) What's genuinely new (5 lines)
1. A crossed-strip steel-shim plus aramid-strap flexure as the skewed lag-pitch coupler (the concept is patented; this design and model are unpublished).
2. The mirrored-flexure "wrong-side" loading asymmetry, and precone chosen specifically to balance the two blades.
3. Governor-mechanism interaction (mode pumping) and the 1-rev moving-average rpm filter fix inside a stock ESC.
4. Tuning the elastic mechanism mode relative to 1/rev for authority, plus on-ESC per-unit phase/gain calibration.
5. Sensorless-ESC operation and scaling below 10 cm (2-3").

## (b) Five papers to read first
1. Paulos & Yim, JGCD 2018 (model, governor as k_m/c_m, flap-hinge rationale).
2. Paulos & Yim, AIAA SciTech 2018, scalability (10 cm rotor, scaling laws, motor sysID).
3. Leines Artieda, Stewart & Benedict, VFS 81 2025 (RCAS model, 180 deg phase, wind tunnel).
4. Chen et al. (HKU), arXiv 2309.13559, 2023 (stock ESC/DShot implementation, phase calibration).
5. US9914535 and US11345469 claims (flexure embodiments; freedom-to-operate).

## (c) Prior results that cut against our approach
- **Flap freedom:** Paulos 2018, Liang 2025 and Tang 2024 all found that without an explicit flap hinge the mirrored coupling forces
  unmatched 2/rev flapping, root moments and vibration. A flexure that is stiff in flap inherits this. Precone balances only the
  *steady* load and does not address the 2/rev kinematic mismatch. This needs checking in our sim.
- **Precone stability:** Ormiston & Bousman (NASA 1972; search snippet) found precone strongly destabilising for hingeless flap-lag
  stability. Check flap-lag damping after adding precone.
- **Sensorless ESCs:** Bird reports a stock sensorless ESC losing sync above about 50 pulses/s. Every published working system uses an encoder
  (Paulos, HKU, Bird, Vertiq has its own). Our AM32 sensorless path is unproven at 7"/10" rpm, and more so at 2-3".
- **Near-resonance operation:** published rotors run above the lag mode (180 deg phase, TAMU) and see friction-induced phase
  scatter. Tuning near 1/rev raises gain but also sensitivity to rpm and per-unit variation, so calibration becomes mandatory.

Gaps in this review: Tang 2024 and JFS 2025 were snippet-only (paywalled); the 2013 IROS paper and the thesis were not read in full;
classic references marked "memory" were not re-verified online.
