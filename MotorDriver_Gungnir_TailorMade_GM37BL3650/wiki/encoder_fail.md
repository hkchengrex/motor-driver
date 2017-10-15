# Encoder Failure

## Failure detection
A encoder failure count will be accumulated if full PWM is given but the encoder count does not change. If encoder count is zero, the malfunction flag will be raised when the count is larger than 0.10 second. If it is non-zero, the flag will be raised when the count is larger than 0.25 second. 

## Auto recovery
When the encoder failure flag is raised, it will perform encoder auto-recovery every 1/8 second. In each recovery attempt, it will give 1/8 of maximum PWM for one control cycle. Encoder failure flag will be lowered immediately if it reads any changes in encoder value.

## Failure signal
When encoder failure flag is raised, it will give out two kinds of signal: 
1. Via CAN. It will flag the feedback signal with a encoder malfunction flag.
2. With life signal LED. The life signal LED will flash much faster.
