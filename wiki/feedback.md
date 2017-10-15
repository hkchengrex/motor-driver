# Continuous Feedback

## CAN feedback
Continuous CAN feedback consist of one packet, 8 bytes in total. It contains 3 pieces of information:
1. Encoder normal/malfunction
2. Encoder count (4 bytes)
3. PWM (3 bytes)

Other kinds of feedback are also sent via CAN. They are separately mentioned in the related files.

## UART feedback
Continuous UART feedback is provided via UART1. It consists of five signed 32 bit integers, separated with a tab, then a new line character. The five numbers are in the following order: 
1. Current encoder count
2. Current encoder velocity
3. Target encoder count
4. Target encoder velocity
5. Current PWM
