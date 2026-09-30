------------------------------- MODULE Tmp108Driver -------------------------------
(***************************************************************************)
(* A formal specification of the `tmp108` Rust driver, composed with the    *)
(* chip model in Tmp108Hw.                                                  *)
(*                                                                         *)
(* Composition is by shared variables: this module EXTENDS Tmp108Hw and its *)
(* Next relation disjoins the chip's autonomous behaviour (drift,           *)
(* conversion) with the driver's steps.  Each driver step performs AT MOST  *)
(* ONE complete I2C transaction, applied atomically -- `write_read` is      *)
(* atomic from embedded-hal's perspective, so a finer grain would model     *)
(* something no caller can observe.                                         *)
(*                                                                         *)
(* The driver is sequential by construction: every method takes `&mut       *)
(* self`, so exactly one operation is ever in flight.                       *)
(*                                                                         *)
(* SCOPE.  Three sources of non-determinism are modelled -- ambient drift,  *)
(* I2C failure, and future cancellation.  A second bus master is NOT        *)
(* modelled; every modify() is a non-atomic read-modify-write               *)
(* (src/lib.rs:13-23) and a second master would find that, but the result   *)
(* would be a documented hazard rather than a defect.  Real time is not     *)
(* modelled, so the one-shot poll budget is a COUNT here and this spec      *)
(* cannot say whether 8 x 5 ms is physically sufficient.                    *)
(***************************************************************************)
EXTENDS Integers, Tmp108Hw

CONSTANTS
    OneShotPolls,    \* poll budget; the driver's real value is 8 (src/lib.rs:607)
    DriverConfigs,   \* configurations a caller may ask configure() to install
    DriverLimits,    \* values a caller may pass to set_low_limit/set_high_limit
    EnabledOps       \* which operations the profile exercises

ASSUME OneShotPolls \in Nat \ {0}
ASSUME DriverLimits \subseteq Temps

(* Concrete configuration sets for the profiles to select with `<-`.        *)
(* TLC's configuration-file parser cannot express a record literal, so a    *)
(* set of Configs has to be named here rather than written in the .cfg.     *)
(*                                                                         *)
(* Varying only the thermostat mode is deliberate: TM is the one setting    *)
(* that changes the driver's behaviour, because it selects between the      *)
(* comparator and interrupt arms of wait_for_alert.  POL only inverts a pin *)
(* level the model reads through AlertAsserted, CR only sets a delay and    *)
(* time is not modelled, and HYS is exercised by Tmp108Hw.cfg.              *)
CfgComparator == [tm |-> 0, pol |-> 0, cr |-> 1, hys |-> 1]
CfgInterrupt  == [tm |-> 1, pol |-> 0, cr |-> 1, hys |-> 1]

BothThermostatModes == { CfgComparator, CfgInterrupt }

--------------------------------------------------------------------------------
(* The word probe() compares against.                                       *)
(*                                                                         *)
(* This is a LITERAL here because it is a literal in the driver:            *)
(*   src/lib.rs:1769   Ok(u16::from_le_bytes(raw) == 0x1022)                *)
(*                                                                         *)
(* The chip's actual reset value is the CONSTANT PorConfigWord.  Keeping    *)
(* the two apart is the whole mechanism of Finding 1: set PorConfigWord to  *)
(* the measured 0x1026 and ProbeDetectsPristineChip fails.                  *)
DriverProbeWord == 4130      \* 0x1022

--------------------------------------------------------------------------------
(* Alert causes.  src/lib.rs:147-163.                                       *)

CauseNone      == "none"
CauseBelowLow  == "below"
CauseAboveHigh == "above"
CauseBoth      == "both"
CauseUnknown   == "unknown"

Causes == { CauseBelowLow, CauseAboveHigh, CauseBoth, CauseUnknown }

(* ops::interrupt_alert_cause, src/lib.rs:530-537.                          *)
(*                                                                         *)
(* Note (FALSE, FALSE) yields Unknown rather than "no event".  The Rust     *)
(* comment at src/lib.rs:519-528 is emphatic that this is INTERPRETATION    *)
(* AFTER QUALIFICATION, not a test for whether an alert happened -- the     *)
(* caller must already have established that one did.  The slow path relies *)
(* on a completed level wait for exactly that.                              *)
AlertCauseOf(l, h) ==
    CASE l /\ h -> CauseBoth
      [] l      -> CauseBelowLow
      [] h      -> CauseAboveHigh
      [] OTHER  -> CauseUnknown

--------------------------------------------------------------------------------
(* Operations and their step labels.                                        *)

OpIdle       == "idle"
OpProbe      == "probe"
OpReadCfg    == "read_configuration"
OpTemp       == "temperature"
OpConfigure  == "configure"
OpShutdown   == "shutdown"
OpSetLow     == "set_low_limit"
OpSetHigh    == "set_high_limit"
OpOneShot    == "one_shot"
OpContinuous == "continuous"
OpWaitAlert  == "wait_for_alert"

AllOps == { OpProbe, OpReadCfg, OpTemp, OpConfigure, OpShutdown,
            OpSetLow, OpSetHigh, OpOneShot, OpContinuous, OpWaitAlert }

ASSUME EnabledOps \subseteq AllOps

--------------------------------------------------------------------------------
VARIABLES
    op,            \* operation in flight, or OpIdle
    step,          \* position within that operation
    polls,         \* one_shot: polls consumed so far
    cause,         \* wait_for_alert: the cause being acquired
    sampleMode,    \* the M field sampled by a read-modify-write
    deferredErr,   \* continuous: the closure failed, but cleanup must still run
    pending,       \* interrupt_sample_pending, src/lib.rs:1244
    retOp,         \* which operation last completed
    retOk,         \* ...and whether it succeeded
    retProbe,      \* probe's boolean result
    retStale,      \* one_shot: the reading it returned was NOT fresh
    retCancelled,  \* the last operation was cancelled rather than completed
    retStep,       \* the step a cancellation interrupted, 0 if none
    everWritten    \* auxiliary: any register write has occurred

dVars == << op, step, polls, cause, sampleMode, deferredErr, pending,
            retOp, retOk, retProbe, retStale, retCancelled, retStep,
            everWritten >>

allVars == << ambient, mode, tm, pol, cr, hys, fl, fh, tlow, thigh,
              tempReg, converting, tempFresh, flagsCurrent,
              op, step, polls, cause, sampleMode, deferredErr, pending,
              retOp, retOk, retProbe, retStale, retCancelled, retStep,
              everWritten >>

--------------------------------------------------------------------------------
(* Helpers.                                                                 *)

(* A configuration write that changes ONLY the mode field, preserving the   *)
(* settings.  This is what shutdown(), continuous() and both of one_shot()'s *)
(* writes do: they use the generated set_m on a sampled register.           *)
(*                                                                         *)
(* Reading tm/cr/hys/pol unprimed rather than from a stored sample is       *)
(* exactly equivalent here, because no modelled agent other than a          *)
(* configuration write can change them, and the driver is sequential.  Only *)
(* mode and the flags change autonomously, which is why sampleMode IS       *)
(* carried.                                                                 *)
SetModeOnly(m) ==
    WriteConfig(m + tm * Pow2[2] + cr * Pow2[5] + hys * Pow2[12] + pol * Pow2[15])

(* The same write, but leaving tempFresh to the caller.  Used only by       *)
(* one_shot's trigger, which must arm the freshness marker in the same step *)
(* as the write that starts the conversion.                                 *)
SetModeOnlyCore(m) ==
    WriteConfigCore(m + tm * Pow2[2] + cr * Pow2[5] + hys * Pow2[12] + pol * Pow2[15])

(* configure(): install a caller's settings, standing down an in-flight     *)
(* one-shot.  ops::apply_config, src/lib.rs:562-579.                        *)
ApplyCallerConfig(m, cfg) ==
    WriteConfig((IF m = ModeOneShot THEN ModeShutdown ELSE m)
                + cfg.tm  * Pow2[2]
                + cfg.cr  * Pow2[5]
                + cfg.hys * Pow2[12]
                + cfg.pol * Pow2[15])

(* Driver-local bookkeeping that most steps leave alone.  retProbe and      *)
(* retStale are deliberately NOT included: they are set by the specific     *)
(* steps that produce them, and listing them here as well would specify the *)
(* same variable twice.                                                     *)
KeepRet == UNCHANGED << retOp, retOk, retCancelled, retStep >>

Finish(o, ok) ==
    /\ op'   = OpIdle
    /\ step' = 0
    /\ retOp' = o
    /\ retOk' = ok
    /\ retCancelled' = FALSE
    /\ retStep' = 0

ClearScratch ==
    /\ polls'       = 0
    /\ cause'       = CauseNone
    /\ sampleMode'  = 0
    /\ deferredErr' = FALSE

--------------------------------------------------------------------------------
Init ==
    /\ HwInit
    /\ op    = OpIdle
    /\ step  = 0
    /\ polls = 0
    /\ cause = CauseNone
    /\ sampleMode = 0
    /\ deferredErr = FALSE
    /\ pending = CauseNone
    /\ retOp = "none"
    /\ retOk = FALSE
    /\ retProbe = FALSE
    /\ retStale = FALSE
    /\ retCancelled = FALSE
    /\ retStep = 0
    /\ everWritten = FALSE

--------------------------------------------------------------------------------
(* Starting an operation.                                                   *)
(*                                                                         *)
(* Clearing retOp here is what scopes the outcome invariants: a property    *)
(* such as ContinuousOkImpliesShutdown speaks about the window between an   *)
(* operation completing and the next one starting.  Without this the        *)
(* invariant would be falsified by a LATER operation legitimately changing  *)
(* the mode.                                                                *)

StartOp(o) ==
    /\ op = OpIdle
    /\ o \in EnabledOps
    /\ op' = o
    /\ step' = 1
    /\ ClearScratch
    /\ retOp' = "none"
    /\ retOk' = FALSE
    /\ retProbe' = FALSE
    /\ retStale' = FALSE
    /\ retCancelled' = FALSE
    /\ retStep' = 0
    /\ UNCHANGED << pending, everWritten >>
    /\ UNCHANGED hwVars

Start == \E o \in AllOps : StartOp(o)

--------------------------------------------------------------------------------
(* Single-transaction operations.                                           *)

(* probe(): one read of the configuration register, compared whole.         *)
(* src/lib.rs:1767-1770.  Note this read ACKNOWLEDGES a latched alert       *)
(* (datasheet.txt:926) -- a side effect probe's documentation does not      *)
(* mention (Finding 3).                                                     *)
ProbeRead ==
    /\ op = OpProbe /\ step = 1
    /\ \/ /\ ReadRegister(RegConfig)
          /\ retProbe' = (ConfigWord = DriverProbeWord)
          /\ Finish(OpProbe, TRUE)
       \/ /\ UNCHANGED hwVars                      \* bus error
          /\ retProbe' = FALSE
          /\ Finish(OpProbe, FALSE)
    /\ ClearScratch
    /\ UNCHANGED << pending, everWritten >>
    /\ UNCHANGED << retStale >>

(* read_configuration() and temperature(): one read each.                   *)
(* src/lib.rs:1792, src/lib.rs:1869.                                        *)
SimpleRead(o, r) ==
    /\ op = o /\ step = 1
    /\ \/ /\ ReadRegister(r) /\ Finish(o, TRUE)
       \/ /\ UNCHANGED hwVars /\ Finish(o, FALSE)
    /\ ClearScratch
    /\ UNCHANGED << pending, everWritten, retProbe, retStale >>

(* set_low_limit()/set_high_limit(): a bare write, with NO preceding read.  *)
(* src/lib.rs:2185, src/lib.rs:2238 -- the closure replaces the whole       *)
(* fieldset, so the register's reset value is never consulted.              *)
SetLimit(o, r) ==
    /\ op = o /\ step = 1
    /\ \/ /\ \E v \in DriverLimits : WriteLimit(r, v)
          /\ everWritten' = TRUE
          /\ Finish(o, TRUE)
       \/ /\ UNCHANGED hwVars /\ UNCHANGED everWritten /\ Finish(o, FALSE)
    /\ ClearScratch
    /\ UNCHANGED << pending, retProbe, retStale >>

--------------------------------------------------------------------------------
(* Read-modify-write operations.                                            *)
(*                                                                         *)
(* Two transactions, not one.  The gap between them is real: a conversion   *)
(* can complete in it, changing mode and the flags.  This is what makes     *)
(* configure() able to stand on a one-shot completion.                      *)

ModifyRead(o) ==
    /\ op = o /\ step = 1
    /\ \/ /\ ReadRegister(RegConfig)
          /\ sampleMode' = mode
          /\ step' = 2
          /\ UNCHANGED op
          /\ KeepRet
       \/ /\ UNCHANGED hwVars
          /\ sampleMode' = 0
          /\ Finish(o, FALSE)
    /\ UNCHANGED << polls, cause, deferredErr, pending, everWritten,
                    retProbe, retStale >>

ConfigureWrite ==
    /\ op = OpConfigure /\ step = 2
    /\ \/ /\ \E cfg \in DriverConfigs : ApplyCallerConfig(sampleMode, cfg)
          /\ everWritten' = TRUE
          /\ Finish(OpConfigure, TRUE)
       \/ /\ UNCHANGED hwVars /\ UNCHANGED everWritten
          /\ Finish(OpConfigure, FALSE)
    /\ ClearScratch
    /\ UNCHANGED << pending, retProbe, retStale >>

ShutdownWrite ==
    /\ op = OpShutdown /\ step = 2
    /\ \/ /\ SetModeOnly(ModeShutdown)
          /\ everWritten' = TRUE
          /\ Finish(OpShutdown, TRUE)
       \/ /\ UNCHANGED hwVars /\ UNCHANGED everWritten
          /\ Finish(OpShutdown, FALSE)
    /\ ClearScratch
    /\ UNCHANGED << pending, retProbe, retStale >>

--------------------------------------------------------------------------------
(* one_shot().  src/lib.rs:2003-2034.                                       *)
(*                                                                         *)
(* The protocol, and its transaction timeline, is pinned in Rust by         *)
(* expected_one_shot_timeline (src/lib.rs:4313-4335):                       *)
(*                                                                         *)
(*   R(1) W(1)  Delay  R(1)  R(1) W(1)  [Delay R(1)]*  R(0)                 *)
(*    prepare          check  trigger      poll        read                 *)
(*                                                                         *)
(* Steps 1-2 prepare (force shutdown), 3 re-reads to CHECK shutdown was     *)
(* reached, 4-5 trigger, 6 polls, 7 reads the result.                       *)
(*                                                                         *)
(* Note steps 3 and 4 read register 1 back to back with nothing between.    *)
(* That redundancy is real and is pinned by the Rust timeline; given        *)
(* datasheet.txt:926 it costs one extra alert acknowledgment per one-shot.  *)
(*                                                                         *)
(* NO ERROR PATH PERFORMS CLEANUP.  src/lib.rs:1930-1955 states this        *)
(* plainly: a Timeout leaves the trigger standing, an UnexpectedMode leaves *)
(* the part in a continuous mode.                                           *)

OneShotPrepareWrite ==
    /\ op = OpOneShot /\ step = 2
    /\ \/ /\ SetModeOnly(ModeShutdown)
          /\ everWritten' = TRUE
          /\ step' = 3 /\ UNCHANGED op /\ KeepRet
       \/ /\ UNCHANGED hwVars /\ UNCHANGED everWritten
          /\ Finish(OpOneShot, FALSE)
    /\ UNCHANGED << polls, cause, sampleMode, deferredErr, pending,
                    retProbe, retStale >>

(* ops::check_prepared, src/lib.rs:661-668: only raw M = 00 may proceed.    *)
(* Anything else aborts WITHOUT triggering and without cleanup.             *)
OneShotCheck ==
    /\ op = OpOneShot /\ step = 3
    /\ \/ /\ ReadRegister(RegConfig)
          /\ IF mode = ModeShutdown
               THEN /\ step' = 4 /\ UNCHANGED op /\ KeepRet
               ELSE Finish(OpOneShot, FALSE)
       \/ /\ UNCHANGED hwVars
          /\ Finish(OpOneShot, FALSE)
    /\ UNCHANGED << polls, cause, sampleMode, deferredErr, pending,
                    everWritten, retProbe, retStale >>

OneShotTriggerRead ==
    /\ op = OpOneShot /\ step = 4
    /\ \/ /\ ReadRegister(RegConfig)
          /\ step' = 5 /\ UNCHANGED op /\ KeepRet
       \/ /\ UNCHANGED hwVars /\ Finish(OpOneShot, FALSE)
    /\ UNCHANGED << polls, cause, sampleMode, deferredErr, pending,
                    everWritten, retProbe, retStale >>

(***************************************************************************)
(* The trigger write also ARMS the freshness marker.                        *)
(*                                                                         *)
(* tempFresh goes FALSE here and is set TRUE by FinishConversion.  So       *)
(* "the temperature register holds a conversion that completed after this   *)
(* trigger" is exactly tempFresh, and NoStaleOneShot becomes expressible.   *)
(* Without this marker, freshness -- a claim about WHICH conversion         *)
(* produced the value -- cannot be stated at all.                           *)
(***************************************************************************)
OneShotTriggerWrite ==
    /\ op = OpOneShot /\ step = 5
    /\ \/ /\ SetModeOnlyCore(ModeOneShot)
          /\ tempFresh' = FALSE
          /\ everWritten' = TRUE
          /\ step' = 6 /\ polls' = 0 /\ UNCHANGED op /\ KeepRet
       \/ /\ UNCHANGED hwVars /\ UNCHANGED everWritten
          /\ polls' = 0
          /\ Finish(OpOneShot, FALSE)
    /\ UNCHANGED << cause, sampleMode, deferredErr, pending,
                    retProbe, retStale >>

(* ops::classify_poll, src/lib.rs:644-650.  Raw M = 00 is Complete, 01 is   *)
(* Converting, 10 and 11 both abort.                                        *)
OneShotPoll ==
    /\ op = OpOneShot /\ step = 6
    /\ \/ /\ ReadRegister(RegConfig)
          /\ CASE mode = ModeShutdown ->            \* Complete
                     /\ step' = 7 /\ UNCHANGED op /\ UNCHANGED polls /\ KeepRet
               [] mode = ModeOneShot ->             \* still converting
                     IF polls + 1 < OneShotPolls
                       THEN /\ polls' = polls + 1
                            /\ step' = 6 /\ UNCHANGED op /\ KeepRet
                       ELSE /\ polls' = polls        \* budget exhausted: Timeout
                            /\ Finish(OpOneShot, FALSE)
               [] OTHER ->                           \* UnexpectedMode
                     /\ polls' = polls
                     /\ Finish(OpOneShot, FALSE)
       \/ /\ UNCHANGED hwVars /\ UNCHANGED polls
          /\ Finish(OpOneShot, FALSE)
    /\ UNCHANGED << cause, sampleMode, deferredErr, pending,
                    everWritten, retProbe, retStale >>

OneShotReadTemp ==
    /\ op = OpOneShot /\ step = 7
    /\ \/ /\ ReadRegister(RegTemp)
          /\ retStale' = ~tempFresh
          /\ Finish(OpOneShot, TRUE)
       \/ /\ UNCHANGED hwVars
          /\ retStale' = FALSE
          /\ Finish(OpOneShot, FALSE)
    /\ ClearScratch
    /\ UNCHANGED << pending, everWritten, retProbe >>

--------------------------------------------------------------------------------
(* continuous().  src/lib.rs:2645-2662.  Async only.                        *)
(*                                                                         *)
(*   enter (R,W) -> closure -> cleanup (R,W), returning user.and(cleanup)   *)
(*                                                                         *)
(* Cleanup runs on the NORMAL-RETURN path only.  It is not a Drop guard,    *)
(* so cancellation at any suspension point leaves the part converting --    *)
(* documented at src/lib.rs:2600-2608 and asserted below as                 *)
(* CancelledContinuousLeaks.                                                *)
(*                                                                         *)
(* An entry failure returns immediately: the closure is never invoked and   *)
(* NO cleanup runs.                                                         *)

ContinuousEnterWrite ==
    /\ op = OpContinuous /\ step = 2
    /\ \/ /\ SetModeOnly(ModeContinuous)
          /\ everWritten' = TRUE
          /\ step' = 3 /\ UNCHANGED op /\ KeepRet
       \/ /\ UNCHANGED hwVars /\ UNCHANGED everWritten
          /\ Finish(OpContinuous, FALSE)          \* no cleanup on entry failure
    /\ UNCHANGED << polls, cause, sampleMode, deferredErr, pending,
                    retProbe, retStale >>

(* The caller's closure, abstracted: it either succeeds or fails, and may   *)
(* read the temperature register.  Its internal transactions are not        *)
(* interesting; what matters is that cleanup runs either way.               *)
ContinuousClosure ==
    /\ op = OpContinuous /\ step = 3
    /\ \/ /\ ReadRegister(RegTemp) /\ deferredErr' = FALSE
       \/ /\ UNCHANGED hwVars /\ deferredErr' = TRUE
    /\ step' = 4 /\ UNCHANGED op /\ KeepRet
    /\ UNCHANGED << polls, cause, sampleMode, pending, everWritten,
                    retProbe, retStale >>

ContinuousCleanupRead ==
    /\ op = OpContinuous /\ step = 4
    /\ \/ /\ ReadRegister(RegConfig)
          /\ sampleMode' = mode /\ step' = 5 /\ UNCHANGED op /\ KeepRet
       \/ /\ UNCHANGED hwVars /\ sampleMode' = 0
          /\ Finish(OpContinuous, FALSE)
    /\ UNCHANGED << polls, cause, deferredErr, pending, everWritten,
                    retProbe, retStale >>

ContinuousCleanupWrite ==
    /\ op = OpContinuous /\ step = 5
    /\ \/ /\ SetModeOnly(ModeShutdown)
          /\ everWritten' = TRUE
          /\ Finish(OpContinuous, ~deferredErr)
       \/ /\ UNCHANGED hwVars /\ UNCHANGED everWritten
          /\ Finish(OpContinuous, FALSE)
    /\ ClearScratch
    /\ UNCHANGED << pending, retProbe, retStale >>

--------------------------------------------------------------------------------
(* wait_for_alert().  src/lib.rs:1594-1699.                                 *)
(*                                                                         *)
(* The retention state machine, which is the subtlest thing in the driver.  *)
(*                                                                         *)
(*   - A retained obligation is read as a COPY, not taken (src/lib.rs:1597) *)
(*     so that a cancelled retry does not lose it.                          *)
(*   - The obligation is armed BEFORE awaiting the temperature read         *)
(*     (src/lib.rs:1680) and cleared only AFTER that read succeeds          *)
(*     (src/lib.rs:1697), with no await in between.                         *)
(*   - Only the interrupt arms an obligation.  Comparator arms never do,    *)
(*     because a comparator alert is a level, not an event: E9 shows its    *)
(*     flags TRACK the hysteresis band, so a set flag is not evidence that  *)
(*     an excursion happened.                                               *)
(*   - Waits are on the LEVEL, never an edge (src/lib.rs:1008-1024): the    *)
(*     entry read already released the pin, so an edge wait would miss an   *)
(*     alert that is already standing.                                      *)

(* step 1: the retained branch, or the entry configuration read (C0). *)
WaitAlertEntry ==
    /\ op = OpWaitAlert /\ step = 1
    /\ \/ /\ pending # CauseNone                    \* retained: go straight to T
          /\ cause' = pending
          /\ UNCHANGED pending
          /\ step' = 5 /\ UNCHANGED op /\ KeepRet
          /\ UNCHANGED hwVars
       \/ /\ pending = CauseNone
          /\ \/ /\ ReadRegister(RegConfig)          \* C0
                /\ CASE tm = TmComparator ->
                          /\ cause' = CauseUnknown
                          /\ UNCHANGED pending      \* comparator NEVER arms
                          /\ step' = 4
                          /\ UNCHANGED op /\ KeepRet
                     [] fl \/ fh ->                 \* interrupt fast path: no GPIO at all
                          /\ cause' = AlertCauseOf(fl, fh)
                          /\ pending' = AlertCauseOf(fl, fh)   \* arm before awaiting T
                          /\ step' = 5
                          /\ UNCHANGED op /\ KeepRet
                     [] OTHER ->                    \* interrupt slow path
                          /\ cause' = CauseNone
                          /\ UNCHANGED pending
                          /\ step' = 2
                          /\ UNCHANGED op /\ KeepRet
             \/ /\ UNCHANGED hwVars                 \* C0 bus error
                /\ cause' = CauseNone
                /\ UNCHANGED pending
                /\ Finish(OpWaitAlert, FALSE)
    /\ UNCHANGED << polls, sampleMode, deferredErr, everWritten,
                    retProbe, retStale >>

(* step 2: the interrupt slow path's level wait.  Enabled only while the    *)
(* pin is actually asserted -- a level wait, not an edge.                   *)
WaitAlertLevelInterrupt ==
    /\ op = OpWaitAlert /\ step = 2
    /\ AlertAsserted
    /\ step' = 3 /\ UNCHANGED op /\ KeepRet
    /\ UNCHANGED hwVars
    /\ UNCHANGED << polls, cause, sampleMode, deferredErr, pending,
                    everWritten, retProbe, retStale >>

(* step 3: the acknowledging read (C1), which also arms the obligation.     *)
(*                                                                         *)
(* A C1 reporting ZERO flags still qualifies and still delivers, as         *)
(* Unknown; it never re-waits (src/lib.rs:1664-1672, and the Rust test      *)
(* a_zero_flag_c1_still_qualifies_and_never_re_waits).  The completed level *)
(* wait is the qualifying evidence, not the flags.                          *)
WaitAlertAck ==
    /\ op = OpWaitAlert /\ step = 3
    /\ \/ /\ ReadRegister(RegConfig)
          /\ cause' = AlertCauseOf(fl, fh)
          /\ pending' = AlertCauseOf(fl, fh)        \* arm before awaiting T
          /\ step' = 5 /\ UNCHANGED op /\ KeepRet
       \/ /\ UNCHANGED hwVars                       \* C1 bus error: evidence is lost
          /\ cause' = CauseNone
          /\ UNCHANGED pending
          /\ Finish(OpWaitAlert, FALSE)
    /\ UNCHANGED << polls, sampleMode, deferredErr, everWritten,
                    retProbe, retStale >>

(* step 4: the comparator level wait.  No obligation is ever armed here. *)
WaitAlertLevelComparator ==
    /\ op = OpWaitAlert /\ step = 4
    /\ AlertAsserted
    /\ step' = 5 /\ UNCHANGED op /\ KeepRet
    /\ UNCHANGED hwVars
    /\ UNCHANGED << polls, cause, sampleMode, deferredErr, pending,
                    everWritten, retProbe, retStale >>

(***************************************************************************)
(* step 5: read the temperature and settle.                                 *)
(*                                                                         *)
(* The obligation was armed on the way in (src/lib.rs:1680) and is cleared  *)
(* only once this read succeeds (src/lib.rs:1697), in the same step, so no  *)
(* cancellation can slip between resolving the read and discharging the     *)
(* debt.  If the read fails, the acknowledged event is still owed.          *)
(***************************************************************************)
WaitAlertReadTemp ==
    /\ op = OpWaitAlert /\ step = 5
    /\ \/ /\ ReadRegister(RegTemp)
          /\ pending' = CauseNone                   \* settled
          /\ Finish(OpWaitAlert, TRUE)
       \/ /\ UNCHANGED hwVars                       \* T failed: the debt persists
          /\ UNCHANGED pending
          /\ Finish(OpWaitAlert, FALSE)
    /\ ClearScratch
    /\ UNCHANGED << everWritten, retProbe, retStale >>

--------------------------------------------------------------------------------
(* Cancellation.                                                            *)
(*                                                                         *)
(* An async future may be dropped at any suspension point.  No cleanup      *)
(* runs -- that is the defect this models, not an omission.                 *)
(*                                                                         *)
(* pending is deliberately UNCHANGED: src/lib.rs:1597-1604 reads the        *)
(* retained obligation as a copy rather than taking it, precisely so that a *)
(* cancelled attempt does not discard it.                                   *)

Cancel ==
    /\ op # OpIdle
    /\ op' = OpIdle
    /\ step' = 0
    /\ ClearScratch
    /\ retOp' = op
    /\ retOk' = FALSE
    /\ retCancelled' = TRUE
    /\ retStep' = step
    /\ UNCHANGED << pending, everWritten, retProbe, retStale >>
    /\ UNCHANGED hwVars

--------------------------------------------------------------------------------
DriverNext ==
    \/ Start
    \/ ProbeRead
    \/ SimpleRead(OpReadCfg, RegConfig)
    \/ SimpleRead(OpTemp, RegTemp)
    \/ SetLimit(OpSetLow, RegTLow)
    \/ SetLimit(OpSetHigh, RegTHigh)
    \/ ModifyRead(OpConfigure) \/ ConfigureWrite
    \/ ModifyRead(OpShutdown)  \/ ShutdownWrite
    \/ ModifyRead(OpOneShot)   \/ OneShotPrepareWrite \/ OneShotCheck
    \/ OneShotTriggerRead \/ OneShotTriggerWrite \/ OneShotPoll \/ OneShotReadTemp
    \/ ModifyRead(OpContinuous) \/ ContinuousEnterWrite \/ ContinuousClosure
    \/ ContinuousCleanupRead \/ ContinuousCleanupWrite
    \/ WaitAlertEntry \/ WaitAlertLevelInterrupt \/ WaitAlertAck
    \/ WaitAlertLevelComparator \/ WaitAlertReadTemp
    \/ Cancel

(* The chip's autonomous behaviour, which must leave the driver alone. *)
ChipNext ==
    /\ \/ Drift \/ StartConversion \/ FinishConversion
    /\ UNCHANGED dVars

Next == ChipNext \/ DriverNext

Spec == Init /\ [][Next]_allVars
--------------------------------------------------------------------------------
(* Properties.                                                              *)

DriverTypeOK ==
    /\ op     \in AllOps \cup { OpIdle }
    /\ step   \in 0 .. 7
    /\ polls  \in 0 .. OneShotPolls
    /\ cause  \in Causes \cup { CauseNone }
    /\ pending \in Causes \cup { CauseNone }
    /\ sampleMode  \in 0 .. 3
    /\ deferredErr \in BOOLEAN
    /\ retOk       \in BOOLEAN
    /\ retProbe    \in BOOLEAN
    /\ retStale    \in BOOLEAN
    /\ retCancelled \in BOOLEAN
    /\ retStep      \in 0 .. 7
    /\ everWritten  \in BOOLEAN

(***************************************************************************)
(* THE PRIZE.                                                               *)
(*                                                                         *)
(* A successful one_shot never reports a reading from a conversion that     *)
(* predates its own trigger.  This is the defect that motivated issue #60:  *)
(* the old implementation wrote the trigger, delayed a fixed time, and      *)
(* read, which returns the PREVIOUS conversion whenever the delay was too   *)
(* short (datasheet.txt:816 -- the register "stores the output of the most  *)
(* recent conversion"; reading it neither starts nor waits for one).        *)
(*                                                                         *)
(* Expressible only because of tempFresh: freshness is a claim about WHICH  *)
(* conversion produced the value, not about the value itself.               *)
(***************************************************************************)
NoStaleOneShot ==
    (retOp = OpOneShot /\ retOk) => ~retStale

(* A successful continuous() has left the part shut down.  src/lib.rs:2645. *)
(* Scoped to the window before the next operation starts, which is what     *)
(* StartOp's reset of retOp achieves.                                       *)
ContinuousOkImpliesShutdown ==
    (retOp = OpContinuous /\ retOk /\ ~retCancelled) => (mode = ModeShutdown)

(***************************************************************************)
(* A DOCUMENTED DEFECT, ASSERTED AS A TRUTH.                                *)
(*                                                                         *)
(* continuous() has no Drop guard (src/lib.rs:2600-2608), so cancelling at  *)
(* any suspension point after the entry write leaves the part converting    *)
(* and burning current, forever.                                            *)
(*                                                                         *)
(* Asserting it -- rather than asserting the opposite and accepting a       *)
(* failure -- means the day someone adds a guard, this check turns RED and  *)
(* forces the documentation and this spec to be corrected in the same       *)
(* change.  A commented-out check would decay silently.                     *)
(***************************************************************************)
CancelledContinuousLeaks ==
    (retOp = OpContinuous /\ retCancelled /\ retStep >= 3)
        => (mode = ModeContinuous)

(* An acknowledged alert obligation is never silently dropped.              *)
(*                                                                         *)
(* Once armed, `pending` may be cleared ONLY by the step that successfully  *)
(* reads the temperature and reports the event.  In particular a            *)
(* cancellation must not clear it, which is why src/lib.rs:1597 copies the  *)
(* retained cause rather than taking it.                                    *)
ObligationClearedOnlyBySettlement ==
    [][ (pending # CauseNone /\ pending' = CauseNone)
          => (op = OpWaitAlert /\ step = 5 /\ retOk') ]_allVars

(* An obligation is armed only from an interrupt-mode path.  Comparator     *)
(* alerts are levels, not events (E9), so they carry no debt.               *)
ObligationOnlyFromInterrupt ==
    [][ (pending = CauseNone /\ pending' # CauseNone)
          => (op = OpWaitAlert /\ tm = TmInterrupt) ]_allVars

(***************************************************************************)
(* FINDING 1.                                                               *)
(*                                                                         *)
(* probe() on a part that has never been written returns TRUE.              *)
(*                                                                         *)
(* This HOLDS when PorConfigWord is the datasheet's 0x1022 and FAILS when   *)
(* it is the 0x1026 measured on a real part (E7).  Tmp108Probe.cfg runs the *)
(* failing case deliberately; see docs/tla/README.md.                       *)
(*                                                                         *)
(* Note the flags cannot confound this: at power-on the limit window is the *)
(* whole domain (datasheet.txt:994), so no conversion can latch FL or FH.   *)
(***************************************************************************)
ProbeDetectsPristineChip ==
    (retOp = OpProbe /\ retOk /\ ~everWritten) => retProbe

================================================================================
