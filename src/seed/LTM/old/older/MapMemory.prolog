:- dynamic horizon/2.

%definizione dei landmark della mappa come nodi di un grafo
landmark(red,-6,-6).
landmark(green,6,-3).
landmark(yellow,-7,5).
landmark(worker,6,6).
landmark(rr,-3,-6).
landmark(dg,3,-6).
landmark(lg,0,-1).
landmark(center,-2,0).
landmark(lc,-6,0).
landmark(ry,-4,5).
landmark(lw,0,7).
landmark(dw,0,3).

%definizione dei percorsi tra i landmark come archi
near(red,lc).
near(lc,red).
near(red,rr).
near(rr,red).
near(rr,dg).
near(dg,rr).
near(green,dg).
near(dg,green).
near(green,lg).
near(lg,green).
near(lg,center).
near(center,lg).
near(center,rr).
near(rr,center).
near(center,lc).
near(lc,center).
near(center,dw).
near(dw,center).
near(dw,worker).
near(worker,dw).
near(worker,lw).
near(lw,worker).
near(ry,lw).
near(lw,ry).
near(ry,dw).
near(dw,ry).
near(ry,yellow).
near(yellow,ry).
near(yellow,lc).
near(lc,yellow).

%definizione degli orizzonti di lookahead
horizon(red,1).
horizon(green,1).
horizon(yellow,1).
horizon(worker,1).
horizon(rr,1).
horizon(dg,1).
horizon(lg,1).
horizon(center,1).
horizon(lc,1).
horizon(ry,1).
horizon(lw,1).
horizon(dw,1).

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

%new planner
planNext(Xstart,Ystart,Goal):-
	yield([],_),
	yield([],OldPath),
	getLandmark(Xstart,Ystart,Lstart),
	horizon(Lstart,Horizon),
	position(Goal,Lgoal),
	lookahead(Lstart,Lgoal,Horizon,[Lnext|_],_),
	(member(Lnext,OldPath)->
%APPRENDIMENTO:
		(backReinforcement(OldPath),
		append([Lnext],[],NewPath))
		;			
		(append([Lnext],OldPath,NewPath))),
	landmark(Lnext,Xnext,Ynext),
	yield([Xnext,Ynext,Lnext,Horizon],_),
	yield(NewPath,_).

%old planner
planNext(Xstart,Ystart,Goal,Horizon):-
	yeld([],_),
	getLandmark(Xstart,Ystart,Lstart),
	lookahead(Lstart,Goal,Horizon,[Lnext|_],_),
	landmark(Lnext,Xnext,Ynext),
	yeld([Xnext,Ynext],_).

getLandmark(X,Y,L):-
	findall(Land,landmark(Land,_,_),List),
	nearest(X,Y,List,9999,"null",L).


nearest(_,_,[],_,UpL,UpL).

nearest(X,Y,[Head|Rest],UpDist,UpL,ReturnL):-
	landmark(Head,XH,YH),
	HeadDist is ((XH-X)^2+(YH-Y)^2)^(1/2),
	(HeadDist<UpDist->
		nearest(X,Y,Rest,HeadDist,Head,ReturnL)
		;
		nearest(X,Y,Rest,UpDist,UpL,ReturnL)).

	


%ho immaginato parte del piano solo se
%	mi trovo nello stato desiderato
lookahead(Stop,Stop,_,[],0).

%altrimenti ho immaginato parte del piano solo se
%	ho esaurito la profondità di ricerca
%	restituisco la distanza euclidea dal landmark a goal (euristica)
lookahead(X,Stop,0,[],DistEuclid):-
	distance(X,Stop,DistEuclid).

%altrimenti ho immaginato parte del piano solo se
%	calcolo gli adiacenti al landmark attuale
%	decremento la profondità
%	trovo il landmark a distanza minima dal goal (euristica)
%	calcolo la distanza tra il landmark attuale ed il landmark migliore
%	restituisco la somma tra le 2 distanze
lookahead(X,Stop,Depth,[Min|MinSubPath],Distance):-
	findall(Y,near(X,Y),List),
	NewDepth is Depth-1,
	findmin(X,List,Distance,Stop,NewDepth,[Min|MinSubPath]).


%ho trovato il minimo solo se
%	ho finito gli adiacenti e restituisco distanza "infinita"
findmin(_,[],100,_,_,[none]).

%altrimenti ho trovato il minimo solo se
%	immagino una parte del piano per l'adiacente attuale
%	trovo il minimo tra gli altri adiacenti
%	se l'adiacente attuale è migliore del minimo tra i restanti
%		restituisco il percorso relativo all'adiacente attuale
%	altrimenti
%		restituisco il percorso relativo al minimo dei rimanenti
findmin(X,[Head|Rest],NewDist,Stop,Depth,[Min|Path]):-
	lookahead(Head,Stop,Depth,HeadPath,HeadDist),
	findmin(X,Rest,DistR,Stop,Depth,[RestMin|RestMinPath]),
	distance(X,Head,DistX2H),	
	DistPathHead is DistX2H+HeadDist,	
	(DistPathHead<DistR -> 
		(functor(Min,Head,0),
		NewDist is DistPathHead,
		Path=HeadPath)
		;
		(functor(Min,RestMin,0),
		NewDist is DistR,
		Path=RestMinPath)).

%distanza tra 2 landmark solo se
%	ho le coordinate del primo landmark
%	ho le coordinate del secondo landmark
%	restituisco la distanza euclidea tra e due coordinate
distance(L1,L2,Dist):-
	landmark(L1,X1,Y1),
	landmark(L2,X2,Y2),
	Dist is ((X1-X2)^2+(Y1-Y2)^2)^(1/2).

%ho potenziato il lookahead sul percorso solo se
%	il percorso è terminato
backReinforcement([]).

%altrimenti
%	rimuovi il vecchio orizzonte dal landmark attuale
%	incremento il vecchio orizzonte
%	asserisco il nuovo orizzonte per il landmark attuale
%	ho potenziato il lookahead anche per i restanti landmark
backReinforcement([Land|Rest]):-
	retract(horizon(Land,H)),
	NH is H+1,
	assert(horizon(Land,NH)),
	backReinforcement(Rest).

