------------------------------ MODULE Tmp108CodecCheck ------------------------------
(***************************************************************************)
(* Runs the constant properties of Tmp108Codec under TLC.                   *)
(*                                                                         *)
(* Tmp108Codec itself declares no VARIABLE, so that Tmp108Hw can EXTEND it  *)
(* for its arithmetic without inheriting a state machine.  This module      *)
(* supplies the trivial one-state machine TLC needs in order to report a    *)
(* failing property BY NAME rather than as an anonymous false assumption.   *)
(***************************************************************************)
EXTENDS Tmp108Codec

VARIABLE tick

Init == tick = 0
Next == UNCHANGED tick
Spec == Init /\ [][Next]_tick

================================================================================
