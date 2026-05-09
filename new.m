arb = Arbotix('port', 'COM5', 'nservos', 5)
%arb.setpos(2, -pi/2,120)

arb.getpos()