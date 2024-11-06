%proprietà degli oggetti
color(objRed,red).
color(objYellow,yellow).
color(objGreen,green).
color(worker,blue).

position(objYellow,yellow).
position(worker,worker).
position(objGreen,green).
position(objRed,red).

position(Obj,X,Y):-position(Obj,L),landmark(L,X,Y).

