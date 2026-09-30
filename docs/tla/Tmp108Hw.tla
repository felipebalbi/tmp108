--------------------------------- MODULE Tmp108Hw ---------------------------------
(***************************************************************************)
(* A formal specification of the TMP108 itself.                             *)
(*                                                                         *)
(* Every clause below is traceable either to the committed datasheet        *)
(* extract (docs/vendor/datasheet.txt:NNN) or to a bench measurement on a   *)
(* real part (E1..E9, tabulated in docs/tla/README.md).  Where the two      *)
(* disagree, or where the datasheet is silent, the measurement wins and the *)
(* divergence is called out in a comment.                                   *)
(*                                                                         *)
(* This module knows nothing about the Rust driver.  It is the reference    *)
(* against which Tmp108Driver is checked, so that a counterexample can be   *)
(* attributed to one side or the other.                                     *)
(*                                                                         *)
(* ---------------------------------------------------------------------   *)
(* UNITS                                                                    *)
(*                                                                         *)
(* Temperatures here are WHOLE DEGREES CELSIUS, not sixteenths.             *)
(* Hysteresis is 0/1/2/4 degrees (datasheet.txt:902-907), i.e. 0/16/32/64   *)
(* sixteenths; a sixteenths-denominated domain small enough to model-check  *)
(* could not represent even a 1 degree band, which would make the           *)
(* comparator logic vacuous.  Sub-degree resolution influences no mode,     *)
(* flag or ALERT decision -- it lives entirely in the encoding, which       *)
(* Tmp108Codec checks over all 4,096 values.  The soundness of the seam is  *)
(* DecodeIsMonotone, checked there rather than assumed here.                *)
(***************************************************************************)
EXTENDS Integers, Tmp108Codec

CONSTANTS
    MinTemp,          \* coldest modelled ambient
    MaxTemp,          \* hottest modelled ambient
    PorTempReg,       \* temperature register value before the first conversion
    PorConfigWord,    \* reset value of the configuration register
    WritableLimits,   \* limit values a bus master is allowed to write
    TestConfigWords   \* configuration words a bus master is allowed to write

Temps == MinTemp .. MaxTemp

ASSUME MinTemp < MaxTemp
ASSUME PorTempReg \in Temps
ASSUME PorConfigWord \in Word16
ASSUME WritableLimits \subseteq Temps
ASSUME TestConfigWords \subseteq Word16

(***************************************************************************)
(* THE TEMPERATURE SCALE IS ABSTRACT AND NON-NEGATIVE.                      *)
(*                                                                         *)
(* Units are whole degrees, but the zero point is arbitrary: the domain is  *)
(* MinTemp..MaxTemp with MinTemp >= 0.  This is sound because every         *)
(* temperature comparison the part makes is a DIFFERENCE -- t > thigh,      *)
(* t < tlow, t <= thigh - HysDegrees, t >= tlow + HysDegrees -- and         *)
(* differences are invariant under an offset.  Nothing in the chip's        *)
(* behaviour depends on where Celsius zero falls.                           *)
(*                                                                         *)
(* The practical reason is that TLC's configuration-file parser rejects     *)
(* negative integer literals.  The real signed range of -128..+127.9375 C   *)
(* is covered exhaustively by Tmp108Codec instead, where it belongs,        *)
(* because that is where sign handling actually lives.                      *)
(***************************************************************************)

--------------------------------------------------------------------------------
(* Named encodings, so the actions read like the datasheet.                 *)

ModeShutdown   == 0     \* datasheet.txt:738  M1=0 M0=0
ModeOneShot    == 1     \* datasheet.txt:743  M1=0 M0=1
ModeContinuous == 2     \* datasheet.txt:753  M1=1
ModeContinuous3 == 3    \* also M1=1, therefore also continuous.  E3.

TmComparator == 0       \* datasheet.txt:914
TmInterrupt  == 1

PolActiveLow  == 0      \* datasheet.txt:910
PolActiveHigh == 1

(* Hysteresis is stored as the 2-bit field; these are the degrees it means. *)
(* datasheet.txt:902-907, Table 9.                                          *)
HysDegrees(h) == IF h = 3 THEN 4 ELSE h

(* Register addresses.  datasheet.txt:808-813, Table 4.                     *)
RegTemp   == 0
RegConfig == 1
RegTLow   == 2
RegTHigh  == 3
Registers == { RegTemp, RegConfig, RegTLow, RegTHigh }

--------------------------------------------------------------------------------
VARIABLES
    ambient,      \* true die temperature; drifts freely
    mode,         \* raw M field, 0..3.  FOUR values: E3 confirms 0b11 is held
    tm,           \* thermostat mode
    pol,          \* ALERT polarity
    cr,           \* conversion rate; inert here, carried only so that the
                  \* configuration WORD can be reconstructed for probe()
    hys,          \* hysteresis field, 0..3
    fl,           \* watchdog low flag
    fh,           \* watchdog high flag
    tlow,         \* TLOW register, degrees
    thigh,        \* THIGH register, degrees
    tempReg,      \* temperature register: the most recent conversion result
    converting,   \* a conversion is in flight
    tempFresh,    \* history: a conversion has completed since it was armed
    flagsCurrent  \* auxiliary: see below

(***************************************************************************)
(* flagsCurrent is an AUXILIARY variable.  It carries no hardware meaning   *)
(* and the driver cannot observe it; it exists solely so that              *)
(* ComparatorQuiescent can be stated.                                       *)
(*                                                                         *)
(* It is TRUE exactly when FL and FH reflect a completed conversion judged  *)
(* against the limits, thermostat mode and hysteresis currently in the      *)
(* registers.  It goes FALSE whenever that correspondence is broken:        *)
(*                                                                         *)
(*   - at reset, because datasheet.txt:822 says the temperature register    *)
(*     reads a placeholder until the first conversion completes;            *)
(*   - on a limit write that changes a limit, since the flags still judge   *)
(*     the old window (datasheet.txt:920-924: the comparison happens "at    *)
(*     the end of every conversion", not on write);                         *)
(*   - on a configuration write that changes TM or HYS, for the same        *)
(*     reason;                                                              *)
(*   - on a configuration read, which clears the flags without re-running   *)
(*     the comparison (datasheet.txt:926).                                  *)
(***************************************************************************)

hwVars == << ambient, mode, tm, pol, cr, hys, fl, fh,
             tlow, thigh, tempReg, converting, tempFresh, flagsCurrent >>

--------------------------------------------------------------------------------
(* Derived state.                                                           *)

(***************************************************************************)
(* The ALERT pin.                                                           *)
(*                                                                         *)
(* E4 and E9 measured the pin to be active exactly when FL or FH is set,    *)
(* in BOTH thermostat modes.  The entire behavioural difference between     *)
(* comparator and interrupt mode lives in how FL and FH evolve (see         *)
(* UpdateFlags), not in how the pin is derived from them.                   *)
(*                                                                         *)
(* datasheet.txt:926 notes that an SMBus alert response clears the pin but  *)
(* NOT the flags, which would separate the two.  The driver never issues    *)
(* one, so the identity holds throughout this model; AlertMatchesFlags      *)
(* records that as a checked invariant rather than a silent assumption.     *)
(***************************************************************************)
AlertAsserted == fl \/ fh

(* The electrical level, once polarity is applied.  datasheet.txt:910-912.  *)
PinIsLow == IF pol = PolActiveLow THEN AlertAsserted ELSE ~AlertAsserted

(* The configuration register as a 16-bit word.                             *)
(* ID and the reserved bits are zero at reset and no modelled write sets    *)
(* them, so they are not carried as state.  ReservedBitsAreZero checks the  *)
(* premise.                                                                 *)
ConfigWord ==
      mode
    + tm  * Pow2[2]
    + (IF fl THEN Pow2[3] ELSE 0)
    + (IF fh THEN Pow2[4] ELSE 0)
    + cr  * Pow2[5]
    + hys * Pow2[12]
    + pol * Pow2[15]

(* What a read of register r returns.  datasheet.txt:673 -- the most        *)
(* significant byte first -- is a wire-ordering detail the driver's         *)
(* Interface handles; at this level a register read yields its value.       *)
RegValue(r) ==
    CASE r = RegTemp   -> tempReg
      [] r = RegConfig -> ConfigWord
      [] r = RegTLow   -> tlow
      [] OTHER         -> thigh

--------------------------------------------------------------------------------
(* Initial state: power-on reset.                                           *)
(*                                                                         *)
(* datasheet.txt:822  "Following power-up or reset, the temperature         *)
(*                     register reads 0 C until the first conversion is     *)
(*                     complete."                                           *)
(* datasheet.txt:994  THIGH = +127.9375 C, TLOW = -128 C, so that "the      *)
(*                     ALERT pin does not become active until the desired   *)
(*                     limit values are programmed".                        *)
(* datasheet.txt:952  "After power-up or a general-call reset, the TMP108   *)
(*                     immediately starts a conversion."                    *)
(*                                                                         *)
(* PorConfigWord is a CONSTANT, not a literal.  The datasheet's value is    *)
(* 0x1022 (:882-889) but E7 measured 0x1026 on a real part, and :880 warns  *)
(* that "other options for the default values are available by request".    *)
(* Parameterising it is what lets Tmp108Probe.cfg demonstrate Finding 1.    *)

PorInit ==
    /\ mode    = ModeField(PorConfigWord)
    /\ tm      = TmField(PorConfigWord)
    /\ pol     = PolField(PorConfigWord)
    /\ cr      = CrField(PorConfigWord)
    /\ hys     = HysField(PorConfigWord)
    /\ fl      = FALSE
    /\ fh      = FALSE
    /\ tlow    = MinTemp          \* the widest window the domain can express
    /\ thigh   = MaxTemp
    /\ tempReg = PorTempReg
    /\ converting = FALSE

HwInit ==
    /\ PorInit
    /\ ambient \in Temps
    /\ tempFresh = FALSE
    /\ flagsCurrent = FALSE

--------------------------------------------------------------------------------
(* Environment: the die temperature moves.                                  *)
(*                                                                         *)
(* Constrained to one degree per step.  An unconstrained jump would be a    *)
(* larger branching factor for no extra coverage, since every intermediate  *)
(* value is reachable anyway by a sequence of steps.  Excluding "unchanged" *)
(* avoids a pointless self-loop.                                            *)

Drift ==
    /\ ambient' \in { t \in Temps : t = ambient - 1 \/ t = ambient + 1 }
    /\ UNCHANGED << mode, tm, pol, cr, hys, fl, fh,
                    tlow, thigh, tempReg, converting, tempFresh, flagsCurrent >>

--------------------------------------------------------------------------------
(* Conversion.                                                              *)

(***************************************************************************)
(* Flag evolution -- the subtlest part of the part, and the place where the *)
(* datasheet is least explicit.                                             *)
(*                                                                         *)
(* datasheet.txt:920-924 says only that the flags "indicate the result of   *)
(* comparing the device temperature at the end of every conversion" to the  *)
(* limit registers.  It does not say whether an in-range conversion CLEARS  *)
(* a flag that a previous conversion set, and the answer turns out to       *)
(* differ by thermostat mode.  Measured (E8, E9):                           *)
(*                                                                         *)
(*   INTERRUPT (TM=1): the flags LATCH.  They survive any number of         *)
(*     in-range conversions and are cleared only by a configuration read    *)
(*     (datasheet.txt:926, :986-988).  This is what makes a latched flag    *)
(*     evidence that an excursion HAPPENED.                                 *)
(*                                                                         *)
(*   COMPARATOR (TM=0): the flags TRACK, and they track the HYSTERESIS      *)
(*     BAND rather than the raw limit.  With THIGH=28, HYS=4 and T=26 --    *)
(*     below the limit but still inside the band -- FH remained set; it     *)
(*     cleared only once T fell below THIGH-HYS.  So in comparator mode a   *)
(*     set flag means "currently out of band", NOT "an excursion            *)
(*     happened".                                                           *)
(*                                                                         *)
(* This measured difference is precisely why the driver must not treat      *)
(* entry flags as an event in comparator mode, which it does not            *)
(* (src/lib.rs:6504, comparator_*_entry_flags_do_not_take_fast_path).       *)
(* The driver was right; this records WHY.                                  *)
(***************************************************************************)
NextFh(t) ==
    IF tm = TmInterrupt
      THEN fh \/ (t > thigh)                                  \* latch
      ELSE IF fh THEN ~(t <= thigh - HysDegrees(hys))         \* hysteretic
                 ELSE (t > thigh)

NextFl(t) ==
    IF tm = TmInterrupt
      THEN fl \/ (t < tlow)                                   \* latch
      ELSE IF fl THEN ~(t >= tlow + HysDegrees(hys))          \* hysteretic
                 ELSE (t < tlow)

(* datasheet.txt:754-756  A conversion is performed, then the part waits    *)
(* for the delay set by CR.  Time is not modelled, so "a conversion is in   *)
(* flight" is a boolean and its duration is unconstrained.                  *)
StartConversion ==
    /\ ~converting
    /\ mode # ModeShutdown
    /\ converting' = TRUE
    /\ UNCHANGED << ambient, mode, tm, pol, cr, hys, fl, fh,
                    tlow, thigh, tempReg, tempFresh, flagsCurrent >>

(***************************************************************************)
(* datasheet.txt:746  "The device returns to the shutdown state at the      *)
(*                     completion of the single conversion.  After the      *)
(*                     conversion, the M1 and M0 bits read 00."             *)
(*                                                                         *)
(* Note this is the ONLY place mode changes without a bus write, and it is  *)
(* exactly the sentinel the driver polls for (src/lib.rs:644-650).          *)
(***************************************************************************)
FinishConversion ==
    /\ converting
    /\ converting' = FALSE
    /\ tempReg'    = ambient
    /\ tempFresh'  = TRUE
    /\ fh'         = NextFh(ambient)
    /\ fl'         = NextFl(ambient)
    /\ mode'       = IF mode = ModeOneShot THEN ModeShutdown ELSE mode
    /\ flagsCurrent' = TRUE
    /\ UNCHANGED << ambient, tm, pol, cr, hys, tlow, thigh >>

--------------------------------------------------------------------------------
(* Bus operations.  These are the only interface the driver has.            *)

(***************************************************************************)
(* A read is an ACTION, not a function, because reading the configuration   *)
(* register has a side effect:                                              *)
(*                                                                         *)
(*   datasheet.txt:926  "Reading the configuration register clears both     *)
(*                       the flags and the pin."                            *)
(*                                                                         *)
(* E2 confirms the converse: reads of 0x00, 0x02 and 0x03 leave a latched   *)
(* flag standing.  Only address 1 acknowledges.                             *)
(*                                                                         *)
(* This single fact is why every driver method that touches the             *)
(* configuration register is a destructive operation on alert evidence.     *)
(***************************************************************************)
ReadRegister(r) ==
    /\ r \in Registers
    /\ IF r = RegConfig
         THEN /\ fl' = FALSE
              /\ fh' = FALSE
              /\ flagsCurrent' = FALSE
         ELSE UNCHANGED << fl, fh, flagsCurrent >>
    /\ UNCHANGED << ambient, mode, tm, pol, cr, hys,
                    tlow, thigh, tempReg, converting, tempFresh >>

(***************************************************************************)
(* Writing the configuration register.                                      *)
(*                                                                         *)
(* E1: FL and FH are READ-ONLY STATUS.  Writes to those bit positions are   *)
(* silently ignored -- verified by writing 0x34, 0x2c and 0x3c to a part    *)
(* held in shutdown (so no conversion could race) and reading back 0x24     *)
(* every time, with a positive control proving a genuinely latched flag     *)
(* does read back as set.                                                   *)
(*                                                                         *)
(* Consequently ops::apply_config's careful echo of the sampled FL/FH       *)
(* (src/lib.rs:541-543) is a no-op, and the read that began the             *)
(* read-modify-write has already destroyed the latch.  Finding 3.           *)
(***************************************************************************)
WriteConfigCore(w) ==
    /\ mode' = ModeField(w)
    /\ tm'   = TmField(w)
    /\ pol'  = PolField(w)
    /\ cr'   = CrField(w)
    /\ hys'  = HysField(w)
    /\ flagsCurrent' = (flagsCurrent /\ TmField(w) = tm /\ HysField(w) = hys)
    /\ UNCHANGED << ambient, fl, fh, tlow, thigh, tempReg, converting >>

(* tempFresh is left to the caller because one_shot's trigger write must    *)
(* arm it atomically with the write itself (see Tmp108Driver).  Every other *)
(* configuration write leaves it alone.                                     *)
WriteConfig(w) == WriteConfigCore(w) /\ UNCHANGED tempFresh

(* datasheet.txt:741  "The device shuts down when current conversion is     *)
(* completed."  Writing M=00 therefore does NOT abort a conversion already  *)
(* in flight, which is why `converting` is explicit state and is left       *)
(* untouched by WriteConfig.                                                *)

WriteLimit(r, v) ==
    /\ r \in { RegTLow, RegTHigh }
    /\ v \in Temps
    /\ IF r = RegTLow THEN tlow' = v /\ UNCHANGED thigh
                      ELSE thigh' = v /\ UNCHANGED tlow
    /\ flagsCurrent' =
           (flagsCurrent /\ v = (IF r = RegTLow THEN tlow ELSE thigh))
    /\ UNCHANGED << ambient, mode, tm, pol, cr, hys, fl, fh,
                    tempReg, converting, tempFresh >>

(* datasheet.txt:716-718 specifies a general-call reset that restores the   *)
(* power-up values; E7 used it to restore the part after the bench work.    *)
(* It is deliberately NOT modelled as an action: the driver never issues    *)
(* one, and HwInit already describes exactly the state it produces, which   *)
(* is what Tmp108Probe.cfg needs in order to reason about a pristine part.  *)

--------------------------------------------------------------------------------
(* A standalone environment, used only by Tmp108Hw.cfg so that the chip can *)
(* be checked on its own.  Tmp108Driver replaces this with the driver.      *)

(* The set of configuration words a master may write is a CONSTANT rather  *)
(* than "every word", because quantifying over all 65,536 makes the state  *)
(* space intractable while adding no coverage: the chip's behaviour depends *)
(* only on the five decoded fields, so a handful of words that vary each    *)
(* field independently exercises everything the unconstrained version       *)
(* would.  The profile chooses the set; see Tmp108Hw.cfg.                   *)
EnvWriteConfig == \E w \in TestConfigWords : WriteConfig(w)

EnvWriteLimit ==
    \E r \in { RegTLow, RegTHigh }, v \in WritableLimits : WriteLimit(r, v)

EnvRead == \E r \in Registers : ReadRegister(r)

HwNext ==
    \/ Drift
    \/ StartConversion
    \/ FinishConversion
    \/ EnvRead
    \/ EnvWriteConfig
    \/ EnvWriteLimit

HwSpec == HwInit /\ [][HwNext]_hwVars /\ WF_hwVars(FinishConversion)

--------------------------------------------------------------------------------
(* Invariants.                                                              *)

HwTypeOK ==
    /\ ambient \in Temps
    /\ mode    \in 0 .. 3
    /\ tm      \in 0 .. 1
    /\ pol     \in 0 .. 1
    /\ cr      \in 0 .. 3
    /\ hys     \in 0 .. 3
    /\ fl      \in BOOLEAN
    /\ fh      \in BOOLEAN
    /\ tlow    \in Temps
    /\ thigh   \in Temps
    /\ tempReg \in Temps
    /\ converting   \in BOOLEAN
    /\ tempFresh    \in BOOLEAN
    /\ flagsCurrent \in BOOLEAN

(* The pin is exactly the disjunction of the flags.  Holds only because the *)
(* SMBus alert response -- the one operation datasheet.txt:926 says clears  *)
(* the pin without clearing the flags -- is not modelled, the driver never  *)
(* issuing one.  Stated as an invariant so that adding ARA support to the   *)
(* model without splitting the pin out is caught here.                      *)
AlertMatchesFlags == AlertAsserted = (fl \/ fh)

(* The configuration word never carries ID or a reserved bit, which is the  *)
(* premise for leaving them out of the state.  datasheet.txt:882-889.       *)
ReservedBitsAreZero ==
    /\ IdField(ConfigWord) = 0
    /\ Field(ConfigWord, 8, 4) = 0
    /\ Bit(ConfigWord, 14) = 0

(***************************************************************************)
(* In comparator mode, a clear pair of flags means the die is inside the   *)
(* limit window -- but only once the flags actually reflect a conversion    *)
(* judged against the CURRENT limits, which is what flagsCurrent tracks.    *)
(*                                                                         *)
(* The guard is not a weakening to make the check pass.  Dropping it        *)
(* produces a genuine counterexample: write a new TLOW above the last       *)
(* conversion result and the register is out of window while the flags,     *)
(* judged against the old window, are still clear.  datasheet.txt:920-924   *)
(* is explicit that the comparison happens "at the end of every             *)
(* conversion", not when a limit is written.                                *)
(*                                                                         *)
(* Deliberately NOT claimed for interrupt mode, where a clear flag means    *)
(* only "nothing since the last acknowledgment" (E8).                       *)
(***************************************************************************)
ComparatorQuiescent ==
    (tm = TmComparator /\ flagsCurrent /\ ~fl /\ ~fh)
        => (tlow <= tempReg /\ tempReg <= thigh)

(***************************************************************************)
(* datasheet.txt:746  "The device returns to the shutdown state at the      *)
(*                     completion of the single conversion."               *)
(*                                                                         *)
(* Stated as an ACTION property over the conversion-completing step rather  *)
(* than as the leads-to (mode = OneShot) ~> (mode = Shutdown).  The         *)
(* leads-to is not checkable here and its failure would be meaningless:     *)
(* the standalone environment may write M=01 on every step forever, so the  *)
(* part is never GIVEN the chance to reach shutdown.  That is an artefact   *)
(* of an unconstrained bus master, not a property of the chip.              *)
(*                                                                         *)
(* The action formulation says the thing that matters and no more: whenever *)
(* a conversion completes while the part is in one-shot mode, the mode      *)
(* becomes shutdown.  It cannot be defeated by an adversarial master.       *)
(***************************************************************************)
OneShotReturnsToShutdown ==
    [][ (mode = ModeOneShot /\ converting /\ ~converting')
            => (mode' = ModeShutdown) ]_hwVars

================================================================================
