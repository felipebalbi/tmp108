-------------------------------- MODULE Tmp108Codec --------------------------------
(***************************************************************************)
(* Pure arithmetic shared by the TMP108 hardware and driver specifications. *)
(*                                                                         *)
(* Nothing here has state.  The module exists so that the parts of the      *)
(* driver that are pure functions of a register word can be checked over    *)
(* their ENTIRE domain -- all 65,536 words, all 4,096 representable         *)
(* temperatures -- which costs milliseconds, while the state machines in    *)
(* Tmp108Hw and Tmp108Driver work in a deliberately small abstract domain.  *)
(*                                                                         *)
(* The division of labour mirrors the Rust test suite, where `mod           *)
(* ops_tests` sweeps every word and the mock-based tests pin a handful of   *)
(* traces.                                                                  *)
(*                                                                         *)
(* Citations of the form datasheet.txt:NNN refer to                         *)
(* docs/vendor/datasheet.txt, the committed text extract of SBOS663A.       *)
(* Citations of the form E1..E7 refer to the bench measurements recorded    *)
(* in docs/tla/README.md.                                                   *)
(***************************************************************************)
EXTENDS Integers, FiniteSets

--------------------------------------------------------------------------------
(* Bit plumbing.                                                            *)

Pow2 == [n \in 0 .. 16 |-> 2 ^ n]

Word16      == 0 .. 65535
Sixteenths  == -2048 .. 2047          \* Celsius(i16), src/lib.rs:262

Bit(w, n)             == (w \div Pow2[n]) % 2
Field(w, lo, width)   == (w \div Pow2[lo]) % Pow2[width]

(* Two's-complement reading of a 16-bit word. *)
Signed16(w) == IF w < 32768 THEN w ELSE w - 65536

(* Unsigned 16-bit image of a signed value. *)
Unsigned16(s) == IF s >= 0 THEN s ELSE s + 65536

(***************************************************************************)
(* Floor division -- and why it is spelled out rather than using \div.      *)
(*                                                                         *)
(* TLC's \div TRUNCATES TOWARD ZERO for negative operands: -1 \div 16 is 0. *)
(* TLC's % FLOORS: -1 % 16 is 15.  The two are therefore mutually           *)
(* inconsistent, since a = b*(a \div b) + (a % b) does not hold at -1.      *)
(*                                                                         *)
(* Rust's `>>` on a signed integer is an ARITHMETIC shift, which is floor   *)
(* division.  Using \div directly would silently disagree with the driver   *)
(* for every negative temperature -- 0xFFFF would decode to 0 here and to   *)
(* -1 in Rust (src/lib.rs:3589 pins the Rust side).                         *)
(*                                                                         *)
(* Because a - (a % b) is exactly divisible by b, truncation and flooring   *)
(* agree on it, so this composition is correct whichever convention \div    *)
(* happens to use.  DivIsFloorDivision below checks both halves of this     *)
(* reasoning so the quirk cannot silently change under us.                  *)
(***************************************************************************)
FloorDiv(a, b) == (a - (a % b)) \div b

--------------------------------------------------------------------------------
(* Temperature codec.                                                       *)
(*                                                                         *)
(* datasheet.txt:820  "One LSB equals 0.0625 C."                            *)
(* datasheet.txt:821  "Negative numbers are represented in binary twos      *)
(*                     complement format."                                  *)
(* datasheet.txt:841  Byte 2 bits D3..D0 are zero.                          *)
(*                                                                         *)
(* Rust: Celsius::from_register is `i16::from_be_bytes(raw) >> 4`           *)
(* (src/lib.rs:426-431).  Rust's `>>` on a signed type is an ARITHMETIC     *)
(* shift, i.e. floor division, NOT truncation toward zero.  TLC's \div does *)
(* truncate, so FloorDiv is used instead -- see its comment above.          *)

RegToCelsius(w) == FloorDiv(Signed16(w), 16)

CelsiusToReg(c) == Unsigned16(c * 16)

(* E6: the low nibble of TLOW/THIGH is read-only.  Writing 0x7ff8 stores    *)
(* 0x7ff0; writing 0x1408 stores 0x1400.  Table 11 (datasheet.txt:1004) is  *)
(* correct about writes; the prose at :994 is correct about the RESET       *)
(* value, which really is 0x7FF8.  They describe different things.          *)
WriteMaskLimit(w) == (w \div 16) * 16

--------------------------------------------------------------------------------
(* Configuration-register codec.                                            *)
(*                                                                         *)
(* Bit positions from tmp108.ddsl:24-79, which index the LITTLE-ENDIAN      *)
(* word; the first byte on the wire is the low byte.  Cross-checked against *)
(* datasheet.txt:883-889 (Table 8) and pinned in Rust at src/lib.rs:3258.   *)
(*                                                                         *)
(*   m   1:0     tm  2      fl  3      fh   4                               *)
(*   cr  6:5     id  7      --  11:8   hys  13:12    --  14   pol  15       *)

ModeField(w)  == Field(w, 0, 2)
TmField(w)    == Bit(w, 2)
FlField(w)    == Bit(w, 3)
FhField(w)    == Bit(w, 4)
CrField(w)    == Field(w, 5, 2)
IdField(w)    == Bit(w, 7)
HysField(w)   == Field(w, 12, 2)
PolField(w)   == Bit(w, 15)

(* The four settings the driver's `Config` models (src/lib.rs:91-101).      *)
(* Note it deliberately models neither m, fl, fh, id nor the reserved bits. *)
Configs == [tm: 0 .. 1, pol: 0 .. 1, cr: 0 .. 3, hys: 0 .. 3]

WordToConfig(w) ==
    [tm  |-> TmField(w),
     pol |-> PolField(w),
     cr  |-> CrField(w),
     hys |-> HysField(w)]

(* The bits apply_config must hand back untouched: fl, fh, id, reserved     *)
(* 11:8 and reserved 14.  Equivalent to the 0x4F98 mask asserted in Rust at *)
(* src/lib.rs:3701.                                                         *)
PreservedBits == {3, 4, 7, 8, 9, 10, 11, 14}

(***************************************************************************)
(* ops::apply_config, src/lib.rs:562-579.                                   *)
(*                                                                         *)
(* Two subtleties, both deliberate in the Rust and both reproduced here:    *)
(*                                                                         *)
(*  - A sampled m of 0b01 is stood down to 0b00, so that changing an        *)
(*    unrelated setting cannot re-trigger an in-flight one-shot (issue      *)
(*    #61; datasheet.txt:744-747).                                          *)
(*  - A sampled m of 0b11 is NOT canonicalised to 0b10, even though both    *)
(*    mean continuous (datasheet.txt:753, "M1 = 1").  E3 confirms the part  *)
(*    holds 0b11 and converts in it.                                        *)
(***************************************************************************)
ApplyConfig(w, cfg) ==
    LET m       == ModeField(w)
        stoodDn == IF m = 1 THEN 0 ELSE m
    IN  stoodDn
      + cfg.tm      * Pow2[2]
      + FlField(w)  * Pow2[3]
      + FhField(w)  * Pow2[4]
      + cfg.cr      * Pow2[5]
      + IdField(w)  * Pow2[7]
      + Field(w, 8, 4) * Pow2[8]
      + cfg.hys     * Pow2[12]
      + Bit(w, 14)  * Pow2[14]
      + cfg.pol     * Pow2[15]

--------------------------------------------------------------------------------
(* Properties.  All are constant predicates; the trivial state machine      *)
(* below exists only so TLC reports them by name.                           *)

(***************************************************************************)
(* Guard against a TLA+/Rust mismatch in negative division.                 *)
(*                                                                         *)
(* The first group records TLC's actual, surprising behaviour: \div         *)
(* truncates toward zero while % floors.  They are asserted, not merely     *)
(* commented, so that a future TLC release which fixes the inconsistency    *)
(* fails this check loudly instead of silently changing what the model      *)
(* means.                                                                   *)
(*                                                                         *)
(* The second group is the property the decoder actually needs, and it      *)
(* matches src/lib.rs:3589 (`0xffff -> -1`, not 0).                         *)
(***************************************************************************)
DivIsFloorDivision ==
    \* TLC's \div truncates toward zero for negatives.
    /\  -1  \div 16 =  0
    /\ -17  \div 16 = -1
    \* TLC's % floors.
    /\  -1   %   16 = 15
    /\ -17   %   16 = 15
    \* FloorDiv repairs the inconsistency.
    /\ FloorDiv(  0, 16) =  0
    /\ FloorDiv( 15, 16) =  0
    /\ FloorDiv( 16, 16) =  1
    /\ FloorDiv( -1, 16) = -1
    /\ FloorDiv(-16, 16) = -1
    /\ FloorDiv(-17, 16) = -2

(* Decoding is total: every one of the 65,536 words the bus could present   *)
(* lands inside the representable range.  A driver must not panic on a bus  *)
(* that lies.  Rust: src/lib.rs:3480.                                       *)
DecodeIsTotal ==
    \A w \in Word16 : RegToCelsius(w) \in Sixteenths

(* Every representable temperature survives a round trip.  Rust:            *)
(* src/lib.rs:3491.                                                         *)
RoundTripsCelsius ==
    \A c \in Sixteenths : RegToCelsius(CelsiusToReg(c)) = c

(***************************************************************************)
(* Monotonicity of the decoder.                                             *)
(*                                                                         *)
(* THIS IS THE ASSUMPTION THE WHOLE DEGREES ABSTRACTION RESTS ON.           *)
(* Tmp108Hw compares temperatures in whole degrees; the chip compares raw   *)
(* counts.  Those two orderings agree only if decoding is monotone.         *)
(*                                                                         *)
(* Checked on adjacent pairs, which implies the general case by             *)
(* transitivity, and costs 65,535 evaluations instead of 2^32.              *)
(***************************************************************************)
DecodeIsMonotone ==
    \A s \in -32768 .. 32766 : FloorDiv(s, 16) <= FloorDiv(s + 1, 16)

(* Finding 2.  The unwritable low nibble carries no threshold information,  *)
(* so losing it costs nothing behaviourally -- which is why the 0x7FF8 vs   *)
(* 0x7FF0 discrepancy is a documentation defect and not a functional one.   *)
LimitNibbleReadOnly ==
    \A w \in Word16 : RegToCelsius(WriteMaskLimit(w)) = RegToCelsius(w)

(* datasheet.txt:848-859, Table 7.  Checked in both directions: the word    *)
(* must decode to the tabulated temperature AND the temperature must        *)
(* re-encode to the word.  Rust: src/lib.rs:3567.                           *)
DatasheetTable7 ==
    LET rows == { <<32752, 2047>>,     \* 0x7FF0  +127.9375 C
                  <<25600, 1600>>,     \* 0x6400  +100 C
                  <<20480, 1280>>,     \* 0x5000   +80 C
                  <<19200, 1200>>,     \* 0x4B00   +75 C
                  <<12800,  800>>,     \* 0x3200   +50 C
                  << 6400,  400>>,     \* 0x1900   +25 C
                  <<   64,    4>>,     \* 0x0040    +0.25 C
                  <<    0,    0>>,     \* 0x0000     0 C
                  <<65472,   -4>>,     \* 0xFFC0    -0.25 C
                  <<59136, -400>>,     \* 0xE700   -25 C
                  <<51456, -880>> }    \* 0xC900   -55 C
    IN  \A r \in rows : /\ RegToCelsius(r[1]) = r[2]
                        /\ CelsiusToReg(r[2]) = r[1]

(* datasheet.txt:994.  The documented power-on limits, decoded.  Note that  *)
(* THIGH's reset word carries bit 3, which E6 shows no write can set --     *)
(* the part holds a value the driver cannot produce -- yet it decodes to    *)
(* exactly the same threshold as the word the driver does produce.          *)
PowerOnLimitsDecode ==
    /\ RegToCelsius(32760) = 2047        \* 0x7FF8  +127.9375 C, reset value
    /\ RegToCelsius(32752) = 2047        \* 0x7FF0  +127.9375 C, what a write stores
    /\ RegToCelsius(32768) = -2048       \* 0x8000  -128 C
    /\ WriteMaskLimit(32760) = 32752     \* the reset value is not re-writable

(* apply_config hands back every bit it does not model.  Rust:              *)
(* src/lib.rs:3793.  Swept over all words against the two extreme configs,  *)
(* the same shape the Rust test uses.                                       *)
ExtremeConfigs ==
    { [tm |-> 0, pol |-> 0, cr |-> 0, hys |-> 0],
      [tm |-> 1, pol |-> 1, cr |-> 3, hys |-> 3] }

ApplyConfigPreservesUnmodelled ==
    \A w \in Word16, cfg \in ExtremeConfigs :
        \A n \in PreservedBits : Bit(ApplyConfig(w, cfg), n) = Bit(w, n)

(* The mode mapping is exactly 00->00, 01->00, 10->10, 11->11.  Rust:       *)
(* src/lib.rs:3829 and the guard test at :3880.                             *)
ApplyConfigNormalisesOnlyOneShot ==
    \A w \in Word16, cfg \in ExtremeConfigs :
        ModeField(ApplyConfig(w, cfg)) =
            (IF ModeField(w) = 1 THEN 0 ELSE ModeField(w))

(* apply_config actually installs what it was asked to.  Rust:              *)
(* src/lib.rs:3774.                                                         *)
ApplyConfigInstallsSettings ==
    \A cfg \in Configs :
        \A w \in { 4130, 4134, 0, 65535 } :   \* 0x1022, 0x1026, 0x0000, 0xFFFF
            WordToConfig(ApplyConfig(w, cfg)) = cfg

(* The `Config` type has exactly 64 inhabitants.  Rust pins this at         *)
(* src/lib.rs:3684 with a comment demanding the preserved mask be revisited *)
(* if it ever changes; the same tripwire belongs here.                      *)
ConfigInhabitantCount == Cardinality(Configs) = 64

================================================================================
