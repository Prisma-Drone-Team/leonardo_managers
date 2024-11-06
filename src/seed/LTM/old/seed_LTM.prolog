:-op(500,xfy,.).
:-op(600,fx,-).

%this is to allow natural language commands to be used as tasks!
:-op(550,xfy,\).

%SWI-prolog: in SWI the '.' operator is already used and constrained to take "dict" arguments 
%		..we have to replace it!
:-op(500,xfy,*.*).
.(L, R, L *.* R).

%this means that the following list is a sequence!
:-op(500,fx,$).

:-['Map2Memory.prolog'].

%Server

getSemantics(X):-
	yield([],_),
	schema(X,SemDef,_),
	yield(SemDef,_),!.

%added 02/12/2020 in SEED 4.0
getGoal(X):-
	yield([],_),
	schema(X,_,[]),!,false.

getGoal(X):-
	yield([],_),
	schema(X,_,Goal),
	yield(Goal,_),!.

getConstraint(X):-
	yield([],_),
	constraint(X,Constraint),
	yield(Constraint,_),!.

getEffect(X):-
	yield([],_),
	effect(X,Effect),
	yield(Effect,_),!.

%getSchemaFromGoal(X):-
%	yield([],_),
%	goal(Schema,Goal),
%	subset(X,Goal),
%	yield(Schema,_),!.

%getSchemaFromEffect(X):-
%	yield([],_),
%	includeEffect(Schema,X),
%	yield(Schema,_),!.
getSchemaFromEffect(X):-
	yield([],_),
	findall(Schema,includeEffect(Schema,X),SchemaList),
	yield(SchemaList,_),!.

%includeEffect(Schema,X):-
%	effect(Schema,Effect),
%	subset(X,Effect),
%	ground(Schema),
%	functor(Schema,0,sat),!,false.

includeEffect(Schema,X):-
	result(Schema,Result),
	subset(X,Result),
	ground(Schema).


% Structure of a Schema:
%	a schema has name, list of sub-schemata and goal, all within the schema()
%	predicate. The name can countain parameters that are propagated to the 
%	list and the goal (unified). Each sub-schema countains a default emphasis value
%	and a list of enabling variables (releaer). If the goal is not present, the schema
%	is not teleological.
%		NOTE: the goal within the schema predicate was added 02/12/2020 in SEED 4.0.
% eg:
%	schema( name(Param), [[subschema1(Param),emph,[releaserList]], [subschema2...]...], goal ).

%schemi ASTRATTI
%schema(wanderUntil(X),[[wander,0,[X.asd]], [avoid,0,[t]]]).

schema(gui, [], [], [] ).
schema(map, [], [], [] ).

%LEARNING AVOIDANCE
schema(alive,[
	[sonarStreamLearn,0,["TRUE"]],
	[engineStream,0,["TRUE"]],
	[blobStream,0,["TRUE"]],
	[inputStream,0,["TRUE"]],
	[rosStream,0,["TRUE"]],
	%[show(alive),0,["TRUE"]],
	%[show(alive,less),0,["TRUE"]],
	[show(requestStream),0,["TRUE"]],
	%[gui,0,["TRUE"]],
	%[avoidLearn,0,["TRUE"]],
	[laserStream,0,["TRUE"]],
	[joyStream,0,["TRUE"]],
	[requestStream,0,["TRUE"]] ],
	[],
	[] ).

schema(q,[[forget(alive),0,["TRUE"]]], [], [] ).

schema(avoidLearn,[
	[avoidObstacle,0,[sonar.obstacle]],
	[gotoxy(-6,6,asd),0,["TRUE"]] ],
	[],
	[] ).

schema(avoidObstacle,[
	[goRun,0,["TRUE"]],
	[goFast,0,["TRUE"]],
	[goSlow,0,["TRUE"]],
	[goNot,0,["TRUE"]],
	[goBack,0,["TRUE"]],
	[turnLeftAll,0,["TRUE"]],
	[turnLeft,0,["TRUE"]],
	[turnLeftFore,0,["TRUE"]],
	[turnNot,0,["TRUE"]],
	[turnRightAll,0,["TRUE"]],
	[turnRight,0,["TRUE"]],
	[turnRightFore,0,["TRUE"]] ],
	[],
	[] ).

schema(goRun,[], [], [] ).
schema(goFast,[], [], [] ).
schema(goSlow,[], [], [] ).
schema(goNot,[], [], [] ).
schema(goBack,[], [], [] ).
schema(turnLeftAll,[], [], [] ).
schema(turnLeft,[], [], [] ).
schema(turnLeftFore,[], [], [] ).
schema(turnNot,[], [], [] ).
schema(turnRightAll,[], [], [] ).
schema(turnRight,[], [], [] ).
schema(turnRightFore,[], [], [] ).

schema(learn2cross,[
	[crossRoad,0,["TRUE"]],
	[semaphore,0,["TRUE"]] ],
	[],
	[] ).

schema(crossRoad,[
	[waitSemaphore,0,["TRUE"]],
	[cross,0,["TRUE"]] ],
	[],
	[] ).

schema(waitSemaphore,[
	[stop,0,["TRUE"]] ],
	[],
	[] ).

schema(stop,[
	[turnNot,0,["TRUE"]],
	[goNot,0,["TRUE"]] ],
	[],
	[] ).

schema(cross,[
	[gotoxy(0,5,crossRoad),0,[sempahore1.arrived]],
	[gotoxy(0,-5,crossRoad),0,[sempahore2.arrived]],
	[avoid,0,["TRUE"]] ],
	[],
	[] ).

schema(semaphore,[], [], [] ).

schema(wanderUntilCollide,[
	[wander,0,["TRUE"]],
	[forget(wanderUntilCollide),0,[engine.collide]] ],
	[],
	[] ).

schema(avoidWalk,[
	[wander,0,["TRUE"]],
	[avoid,0,["TRUE"]] ],
	[],
	[] ).

schema(goto(X,Y,ID),[
	[avoid,0,["TRUE"]],
	[gotoxy(X,Y,ID),0,["TRUE"]] ],
	[gotoxy(X,Y,ID).arrived],
	[] ).

schema(followColor(Color),[
	[avoid,2,["TRUE"]],
	[reachColor(Color),0,["TRUE"]] ],
	[Color.reached],
	[] ).

schema(searchColorRandomly(Color),[
	[avoid,0,[-Color.near]],
	[wander,0,[-Color.present]],
	[reachColor(Color),0,[Color.present]] ],
	[],
	[] ).

schema(explorexy(X,Y,ID),[
	[avoid,0,["TRUE"]],
	[gotoxy(X,Y,ID),0,["TRUE"]],
	[wander,0,[gotoxy(X,Y,ID).arrived]] ],
	[],
	[] ).

schema(gotoColor(X,Y,Color),[
	[explorexy(X,Y,gotoColor(X,Y,Color)),0,[-Color.present]],
	[followColor(Color),0,[Color.present]] ],
	[Color.reached],
	[] ).

schema(pickup(Obj),[
	[place(Obj,C,knapsack),0,[-knapsack.full,C.reached]] ],
	[knapsack.Obj],
	[] ):-color(Obj,C).

schema(putdown(Obj),[
	[place(Obj,C,ground),0,[knapsack.Obj]] ],
	[place(Obj,C,ground).done],
	[] ):-color(Obj,C).

schema(take(Obj,X,Y),[
	[gotoColor(X,Y,Color),0,[-knapsack.full,-knapsack.Obj,-Color.reached]],
	[pickup(Obj),0,[Color.reached,-knapsack.full,-knapsack.Obj]] ],
	[place(Obj,Color,knapsack).done],
	[] ):-color(Obj,Color).
	
schema(takeToMe(Obj),[
	[takeObject(Obj),0,["TRUE"]],
	[return(Obj),0,["TRUE"]] ],
	[place(Obj,Color,ground).done],
	[] ):-color(Obj,Color).

schema(takeObject(Obj),[
	[gotoObject(Obj),0,[-knapsack.full,-knapsack.Obj,-Color.reached]],
	[pickup(Obj),0,[Color.reached,-knapsack.full,-knapsack.Obj]] ],
	[knapsack.Obj],
	[] ):-color(Obj,Color).

schema(gotoObject(Obj),[
	[explorexy(X,Y,Obj),0,[-Color.present]],
	[followColor(Color),0,[Color.present]] ],
	[Color.reached],
	[] ):-color(Obj,Color),position(Obj,Land),landmark(Land,X,Y).

schema(return(Obj),[
	[gotoObject(worker),0,[knapsack.Obj]],
	[putdown(Obj),0,[blue.reached]] ],
	[place(Obj,Color,ground).done],
	[] ):-color(Obj,Color).

%questi schemi sono per ICSR
schema(take4,[
	[takeToMe(objRed),0,["TRUE"]],
	[takeToMe(objGreen),0,["TRUE"]],
	[takeToMe(objBrown),0,["TRUE"]],
	[takeToMe(objYellow),0,["TRUE"]] ],
	[],
	[] ).

schema(take3,[
	[takeToMe(objRed),0,["TRUE"]],
	[takeToMe(objGreen),0,["TRUE"]],
	[takeToMe(objYellow),0,["TRUE"]] ],
	[],
	[] ).

schema(take2,[
	[takeToMe(objRed),0,["TRUE"]],
	[takeToMe(objGreen),0,["TRUE"]] ],
	[],
	[] ).

%sequential veresion
schema(takeNseq,$[
	[takeToMe(objRed),0,["TRUE"]],
	[takeToMe(objYellow),0,["TRUE"]],
	[takeToMe(objBrown),0,["TRUE"]],
	[takeToMe(objGreen),0,["TRUE"]] ],
	[],
	[] ).

schema(takeN,[
	[takeToMe(objRed),0,["TRUE"]],
	[takeToMe(objYellow),0,["TRUE"]],
	[takeToMe(objOrange),0,["TRUE"]],
	[takeToMe(objBrown),0,["TRUE"]],
	[takeToMe(objPink),0,["TRUE"]],
	[takeToMe(objGreen),0,["TRUE"]] ],
	[],
	[] ).

schema(take1(X),[
	[taketo(X,worker),0,["TRUE"]] ],
	[],
	[] ).

schema(taketo(Obj,Pos),[
	[take(Obj,Xobj,Yobj),0,[-knapsack.Obj]],
	[gotoColor(Xpos,Ypos,PosColor),0,[knapsack.Obj]],
	[putdown(Obj),0,[knapsack.Obj,PosColor.reached]] ],
	[place(Obj,ObjColor,ground).done],
	[] ):-
		color(Obj,ObjColor),
		color(Pos,PosColor),
		position(Obj,Xobj,Yobj),
		position(Pos,Xpos,Ypos).

schema(routetoColor(Subject),[
	[routeto(Subject),0,[-Color.present]],
	[followColor(Color),0,[Color.present]] ],
	[Color.reached],
	[] ):-color(Subject,Color).

schema(plantake(Obj,Pos),[
	[routetoColor(Obj),0,[-ObjColor.reached,-knapsack.full,-knapsack.Obj]],
	[pickup(Obj),0,[ObjColor.reached,-knapsack.full,-knapsack.Obj]],
	[routetoColor(Pos),0,[-PosColor.reached,knapsack.Obj]],
	[putdown(Obj),0,[knapsack.Obj,PosColor.reached]] ],
	[place(Obj,ObjColor,ground).done,PosColor.present],
	[] ):-
		color(Pos,PosColor),
		color(Obj,ObjColor).

schema(gotopoi(X,Y),[
	[poi(X,Y),0,["TRUE"]],
	[avoid,0,["TRUE"]] ],
	[poi(X,Y).arrived],
	[] ).


%-- tests for speech recognition
schema(go\to\position\Px\Py,[
	[reachpos(Px,Py,0),0,["TRUE"]] ],
	[],
	[] ).

schema(hello,[
	[say("hello"),0,["TRUE"]] ],
	[],
	[] ).

schema(go\to\position\Px\Py\Pw,[
	[reachpos(Px,Py,Pw),0,["TRUE"]] ],
	[],
	[] ).
%--




%ICOSAF prepare kit of components
% kits: k1, k2
% shelves: s1,s2,s3
% components: screw,tube,bar,gear

% vars:
%	K\is\ready
%	C\is\reached
%	S\is\reached
%	L\is\reached
%	C\is\taken
%	C\is\placed\in\K
%	gripper\is\open
%	gripper\is\close
%	gripper\is\free
%	arm\is\in\pose\P
%
%	pose\of\C\is\known
%	pose\of\S\is\known

schema(deliver\Object\from\Shelf,$[
	[take\Object\from\Shelf,0,["TRUE"]],
	[leave\Object\on\poor\from\poor_left,0,[Object\is\placed\on\self]] ],
	[Object\is\placed\on\poor],
	[] ).

%schema(prepare\kit\in\sequence,$[
schema(prepare\kit,$[
	[collect\components\in\sequence,0,["TRUE"]],
	[deliver\components\in\sequence,0,["TRUE"]] ],
	[tube\is\placed\on\poor],
	[] ).

%kit k1
schema(collect\components\for\_,[
	[take\gear\from\shelf1,0,["TRUE"]],
	[take\tube\from\shelf2,0,["TRUE"]],
	[take\bar\from\shelf2,0,["TRUE"]],
	[take\screw\from\shelf3,0,["TRUE"]] ],
	[gear\is\placed\on\self,
	 tube\is\placed\on\self,
	 bar\is\placed\on\self,
	 screw\is\placed\on\self],
	[] ).

schema(collect\components\in\sequence,$[
	[take\gear\from\shelf1,0,["TRUE"]],
	[take\tube\from\shelf2,0,["TRUE"]],
	[take\bar\from\shelf2,0,["TRUE"]],
	[take\screw\from\shelf3,0,["TRUE"]] ],
	[screw\is\placed\on\self],
	[] ).

schema(deliver\components\in\sequence,$[
	[leave\gear\on\poor\from\poor_left,0,["TRUE"]],
	[leave\bar\on\poor\from\poor_left,0,["TRUE"]],
	[leave\screw\on\poor\from\poor_left2,0,["TRUE"]],
	[leave\tube\on\poor\from\poor_right,0,["TRUE"]] ],
	[tube\is\placed\on\poor],
	[] ).

%kit k2

schema(take\Component\from\Shelf,[
	[reach\Component\to\Shelf,0,["TRUE"]],
	[pick\Component\from\Shelf,0,[Component\is\reached]] ],
	[Component\is\placed\on\self], [] ).

schema(leave\Component\on\poor\from\Side,[
	[reach\Side,0,["TRUE"]],
	[place\Component\to\poor,0,[Side\is\reached]] ],
	[Component\is\placed\on\poor], [] ).

% NOTE: how to convert an object into a pose so that the component will
%	be reached instead of the pose?
schema(reach\Component\to\Shelf,[
	[go\to\Shelf,0,["TRUE"]],
	%[go\to\Component\precisely,0,[pose\of\Component\is\known]] ],
	[go\to\Component\precisely,0,[Shelf\is\close]] ],
	[Component\is\reached],
	[] ).

%% ...to avoid impact with the poor robot
schema(reach\poor_right,$[
	[go\to\poor_right2,0,["TRUE"]],
	[go\to\poor_right2\precisely,0,["TRUE"]],
	[go\to\poor_right\precisely,0,["TRUE"]] ],
	[poor_right\is\reached],
	[] ).

schema(reach\Location,[
	[go\to\Location,0,["TRUE"]],
	%[go\to\Component\precisely,0,[pose\of\Component\is\known]] ],
	[go\to\Location\precisely,0,[Location\is\close]] ],
	[Location\is\reached],
	[] ).

schema(go\to\Target\precisely,[
	%[avoid,0,["TRUE"]],
	[goinpos(Target),0,["TRUE"]] ],
	[Target\is\reached],
	[] ).

schema(go\to\Target,[
	[avoid,0,["TRUE"]],
	[gotoxy(Target),0,["TRUE"]] ],
	[Target\is\close],
	[] ).

%schema(move\to\landmark\L,[
%	[avoid,0,["TRUE"]],
%	[goto(L),0,["TRUE"]] ],
%	[L\is\reached] ):-pose(_,L).

schema(pick\Component\from\Shelf,[
	[arm(move,Component,Shelf,self),0,["TRUE"]] ],
	[Component\is\placed\on\self],
	[] ).

schema(place\Component\to\poor,[
	[arm(move,Component,self,poor),0,["TRUE"]] ],
	[Component\is\placed\on\poor],
	[] ).

%not used, for general purpose
schema(move\Object\from\Pose1\to\Pose2,[
	[arm(move,Object,Pose1,Pose2),0,["TRUE"]] ],
	[Object\is\placed\on\Pose2],
	[] ).

%move the arm to a pose
schema(arm(move, Object, _, Pose), [], [Object\is\placed\on\Pose], [] ).
%move the robot to a landmark
schema(goinpos(Pose), [], [Pose\is\reached], [] ).
schema(gotoxy(Pose), [], [Pose\is\close], [] ).
%avoid obstacles (mobile)
%schema(avoid,[], [] ). %already exists


%ICOSAF - end




schema(reachpos(Px,Py,Pw),[
	[laserStream,0,["TRUE"]],
	[avoid,0,["TRUE"]],	
	[goinpos(Px,Py,Pw),0,["TRUE"]] ],
	[],
	[] ).

schema(conflict,[
	[take(objRed,-2,0),0,[-place(objRed,red,ground).done]],
	[take(objGreen,2,0),0,[-place(objGreen,green,ground).done]] ],
	[],
	[] ).

schema(conflict2,[
	[take(objRed,-2,0),0,[-place(objRed,red,ground).done]],
	[gotoColor(6,6,blue),0,[knapsack.objRed]],
	[putdown(objRed),0,[knapsack.objRed,blue.reached]],
	[take(objGreen,2,0),0,[-place(objGreen,green,ground).done]],
	[gotoColor(6,6,blue),0,[knapsack.objGreen]],
	[putdown(objGreen),0,[knapsack.objGreen,blue.reached]] ],
	[],
	[] ).

schema(request,[[takeObject(objGreen),0,["TRUE"]],
	[takeObject(objRed),3,["TRUE"]] ],
	[],
	[] ).


%sequential speak test
schema(sayseq,$[
	[say("this is"),0,["TRUE"]],
	[say("a test"),0,["TRUE"]],
	[say("of sequential"),0,["TRUE"]],
	[say("speech."),0,["TRUE"]] ],
	[],
	[] ).

%longer sequence
schema(ahab,$[
	[say("God help thee, old man, thy thoughts have created a creature in thee."),0,["TRUE"]],
	[say("and he, whose intense thinking thus makes him a Prometheus,"),0,["TRUE"]],
	[say("a vulture feeds upon that heart forever."),0,["TRUE"]],
	[say("the vulture, the very creature he creates."),0,["TRUE"]] ],
	[],
	[] ).

%schemi CONCRETI (ie. behavior)
schema(wander,[],[],[]).
schema(forget(_),[],[],[]).
schema(sonarStream,[],[],[]).
schema(engineStream,[],[],[]).
schema(blobStream,[],[],[]).
schema(laserStream,[],[],[]).
schema(inputStream,[],[],[]).
schema(remember(_),[],[],[]).
schema(listing,[],[],[]).
schema(avoid,[],[],[]).
schema(gotoxy(X,Y,ID), [], [gotoxy(X,Y,ID).arrived],[]).
schema(goinpos(_,_,_),[],[],[]).
%schema(reachColor(_),[],[]).
schema(reachColor(C), [], [C.reached], []):-color(_,C).
%schema(place(Obj,_,knapsack), [], [knapsack.Obj],[]).
schema(place(Obj,Col,Pos), [], [place(Obj,Col,Pos).done],[]).
schema(fg(_),[],[],[]).
schema(emp(_),[],[],[]).
schema(show(_),[],[],[]).
schema(show(_,_),[],[],[]). %is the old showLess
schema(wait,[],[],[]).
schema(requestStream,[],[],[]).
schema(routeto(_),[],[],[]).
schema(test,[],[],[]).
schema(rosStream,[],[],[]).
schema(joyStream,[],[],[]).


schema(ltm(_),[],[],[]).
schema(set(_,_,_),[],[],[]).
schema(set(_,_),[],[],[]). %means only once
schema(compete(_,_,_,_),[],[],[]).
schema(compete(_,_,_),[],[],[]). %means only once

schema(decide(_,_),[],[],[]).
schema(freeze(_),[],[],[]).
schema(audioStream,[[juliusAudioRecognizer,0,["TRUE"]] ],
	[],
	[] ).
%schema(audioStream,[[googleAudioRecognizer,0,["TRUE"]]],[]).
schema(googleAudioRecognizer,[],[],[]).
schema(juliusAudioRecognizer,[],[],[]).
schema(vocalStream,[],[],[]).
schema(say(X), [], [say(X).done],[]).
schema(dialogue(_,_),[],[],[]).

schema(exePlan(_),[],[],[]).
schema(exeSeq(_),[],[],[]).
schema(setPlayer,[],[],[]).
schema(poiStream,[],[],[]).
schema(poi(X,Y), [], [poi(X,Y).arrived],[]).
schema(emp(_),[],[],[]).
schema(cpuMonitor,[],[],[]).
schema(gui,[],[],[]).

schema(randsetPlayer,[],[],[]).

schema(sonarStreamLearn,[],[],[]).
schema(compete(_,_),[],[],[]).

schema(tmPlanner,[],[],[]).

%SAPHARI Airbus Plan-Exe (ape)
schema(ape,[
	[exePlan([search(bracket1),take(bracket1),give(bracket1,worker)]),0,["TRUE"]],
	[installBracket(bracket1,location1),0,["TRUE"]],
	[show(installBracket(bracket1,location1)),0,["TRUE"]] ],
	[],
	[] ).

schema(installBracket(Obj,Pos),[
	%[receive(Obj,C),0,[worker.Obj]],
	[get(Obj),0,[-worker.Obj,-knapsack.full]],
	[give(Obj,worker),0,[knapsack.Obj]],
	[attachBracket(Obj,Pos),0,[knapsack.Obj]] ],
	[Pos.Obj],
	[] ).%:-color(Obj,C).

schema(get(Obj),[
	[take(Obj),0,[-knapsack.full]],
	[searchColor(Color),0,[-knapsack.full,-Color.present]] ],
	[place(Obj,Color,knapsack).done],
	[] ):-color(Obj,Color).

schema(searchColor(C), [], [C.present],[]).

schema(search(Obj),[
	[searchColor(Color),0,["TRUE"]] ],
	[Color.present],
	[] ):-color(Obj,Color).

%schema(search(Obj),[
%	[goto(X1,Y1,search(Obj)),0,["TRUE"]],
%	[goto(X2,Y2,search(Obj)),0,["TRUE"]]
%	]):-landmark(worker,X1,Y1),landmark(yellow,X2,Y2).

schema(take(Obj),[
	[followColor(Color),0,[-knapsack.full,-knapsack.Obj,-Color.reached]],
	[pickup(Obj),0,[Color.reached,-knapsack.full,-knapsack.Obj]] ],
	[place(Obj,Color,knapsack).done],
	[] ):-color(Obj,Color).

schema(give(Obj,worker),[
	[goto(0,0,give(Obj,worker)),0,[-blue.present,knapsack.Obj,-gotoxy(0,0,give(Obj,worker)).near]],
	[handover(Obj,worker),0,["TRUE"]]
	%[bring(Obj,location1),0,["TRUE"]],
	%[bring(Obj,location2),0,["TRUE"]],
	%[bring(Obj,location3),0,["TRUE"]]
	],
	[worker.Obj],
	[] ).%:-landmark(center,X,Y).

schema(attachBracket(Obj,Pos),[
	[bring(Obj,Pos),0,["TRUE"]] ],
	[Pos.Obj],
	[] ).

schema(bring(Obj,Pos),[
	[gotoObject(Pos),0,[knapsack.Obj]],
	[putdown(Obj,Pos),0,[C.reached]] ],
	[Pos.Obj],
	[] ):-color(Pos,C).

schema(handover(Obj,worker),[
	[followColor(blue),0,[knapsack.Obj]],
	[putdown(Obj,worker),0,[blue.reached]] ],
	[worker.Obj],
	[] ).

schema(putdown(Obj,Pos),[
	[place(Obj,C,Pos),0,[knapsack.Obj]] ],
	[place(Obj,C,Pos).done],
	[] ):-color(Pos,C).

schema(receive(_,_),[],[],[]).


%RO-MAN
schema(simuarm,[],[],[]).
schema(simutake(X), [], [X.taked],[]).
schema(makeCoffee,[[simutake(coffee),0,["TRUE"]],
	[simutake(cup),0,["TRUE"]],
	[simutake(sugar),0,["TRUE"]],
	[simutake(spoon),0,["TRUE"]] ],
	[],[]).

schema(makeTea,[[simutake(tea),0,["TRUE"]],
	[simutake(cup),0,["TRUE"]],
	[simutake(sugar),0,["TRUE"]],
	[simutake(spoon),0,["TRUE"]] ],
	[],
	[] ).

schema(roman,[[simuarm,0,["TRUE"]],
        [makeCoffee,0,[-tea.taked]],
        [makeTea,0,[-coffee.taked]],%]).
	[say(en("done, have a good coffee!")),0,[coffee.taked,cup.taked,spoon.taked,sugar.taked]],
	[say(en("done, have a good tea!")),0,[tea.taked,cup.taked,spoon.taked,sugar.taked]] ],
	[],
	[] ).


%elaborazione del linguaggio IT


schema(it("chiudi"),[[forget(alive),0,["TRUE"]]], [],[]).
schema(it("lista"),[[listing,0,["TRUE"]]], [],[]).

schema(note,[[amplify(simutake(coffee),-1),0,["TRUE"]]], [],[]).
schema(nocaffe,[[amplify(simutake(tea),-1),0,["TRUE"]]], [],[]).


schema(it(Sentence),[],[],[]):-
	string(Sentence),
	split_string(Sentence," ","",[_|[]]).

schema(it(Sentence),[
	[it(Word),0,["TRUE"]],
	[it(SubSentence),0,["TRUE"]]], [], []):-
		string(Sentence),
		split_string(Sentence," ","",[Word|Rest]),
		join_string(Rest," ",SubSentence).

schema(saySequence(it(Sentence)),[[say(it(Sentence)),0,["TRUE"]]], [], []):-
	string(Sentence),
	split_string(Sentence," ","",[_|[]]).

schema(saySequence(it(Sentence)),[
	[say(it(Word)),0,["TRUE"]],
	[saySequence(it(SubSentence)),0,["TRUE"]]], [], []):-
		string(Sentence),
		split_string(Sentence," ","",[Word|Rest]),
		join_string(Rest," ",SubSentence).

%schema(segment(Seg),[]):-!,is_list(Seg).

schema(saySegment,[
	[segment([say(it("sopra")),say(it("la")),say(it("panca")),say(it("la")),say(it("capra")),say(it("campa"))]),0,["TRUE"]] ], 
	[],
	[] ).

schema(segment([Head|[]]),[[Head,0,["TRUE"]]], [],[]).

schema(segment([Head|Rest]),[
	[Head,0,["TRUE"]],
	[segment(Rest),0,HeadGoal] ], [], []):-
		goal(Head,HeadGoal).

schema(segment([Head|Rest]),[
	[Head,0,["TRUE"]],
	[segment(Rest),0,["TRUE"]] ],
	[],
	[] ).

schema(amplify(S,_),[],[],[]):-
		schema(S,_).

schema(adm,[],[],[]).
schema(hai,[],[],[]).


%schemi SAPHARI

schema(saphari,[
	[gestureRecognition,0,["TRUE"]],
	[objectRecognition,0,["TRUE"]],
	%[audioStream,0,["TRUE"]],
	[fusionEngine,0,["TRUE"]] ],
	[],
	[] ).

schema(gesture(take),[],[],[]).
schema(gesture(give),[],[],[]).
schema(gesture(stop),[],[],[]).
schema(gesture(no),[],[],[]).
schema(gesture(leaves),[],[],[]).
schema(gesture(point),[],[],[]).
schema(gesture(come),[],[],[]).
schema(gesture(search),[],[],[]).
%schema(gesture(pointleft),[],[]).
%schema(gesture(pointfront),[],[]).

schema(speech(take),[],[],[]).
schema(speech(give),[],[],[]).
schema(speech(stop),[],[],[]).
schema(speech(no),[],[],[]).
schema(speech(leaves),[],[],[]).
schema(speech(come),[],[],[]).
schema(speech(search),[],[],[]).

schema(speech(next),[],[],[]).
schema(speech(done),[],[],[]).
schema(speech(ok),[],[],[]).


schema(object(cleanbottle),[],[],[]).

schema(object(cloth),[],[],[]).
schema(object(screw),[],[],[]).
schema(object(screwdriver),[],[],[]).
schema(object(plate),[],[],[]).
schema(object(glue),[],[],[]).
schema(object(null),[],[],[]).

schema(vrepExec(_,_),[],[],[]).

schema(fusionEngine,[],[],[]).

schema(gestureRecognition,[],[],[]).
%schema(gestureRecognition,[
%	[gesture(point),0,["TRUE"]],
%	[gesture(take),0,["TRUE"]],
%	[gesture(come),0,["TRUE"]],
%	[gesture(leaves),0,["TRUE"]],
%	[gesture(no),0,["TRUE"]],
%	[gesture(give),0,["TRUE"]],
%	[gesture(search),0,["TRUE"]],
%	[gesture(stop),0,["TRUE"]]],[]).

schema(objectRecognition,[
	[object(null),1.5,["TRUE"]] ],
	[],
	[] ).
%schema(objectRecognition,[
%	[object(cloth),0,["TRUE"]],
%	[object(screw),0,["TRUE"]],
%	[object(plate),0,["TRUE"]],
%	[object(glue),0,["TRUE"]],
%	[object(screwdriver),0,["TRUE"]]],[]).

%elaborazione del linguaggio per SAPHARI
schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]], [], []):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,Noun,Object]),
		term_string(TermAction,Action),
		term_string(me,Noun),
		term_string(TermObject,Object).

	%oggetti composti (eg. screwdriver)
schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]], [], []):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,"me",Ob_pt1,Ob_pt2]),
		term_string(TermAction,Action),
		concat_string([Ob_pt1,Ob_pt2],Object),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]], [], []):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,Ob_pt1,Ob_pt2]),
		term_string(TermAction,Action),
		concat_string([Ob_pt1,Ob_pt2],Object),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]], [], []):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,Object]),
		term_string(TermAction,Action),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]]], [], []):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action]),
		term_string(TermAction,Action),
		schema(speech(TermAction),_).

% i prossimi 2 sono aggiunti prima di tolosa
schema(en(Sentence),[
	[object(TermObject),2,["TRUE"]]], [], []):-
		string(Sentence),
		split_string(Sentence," ","",[_,Ob_pt1,Ob_pt2]),
		concat_string([Ob_pt1,Ob_pt2],Object),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[object(TermObject),2,["TRUE"]]], [], []):-
		string(Sentence),
		split_string(Sentence," ","",[_,Object]),
		term_string(TermObject,Object).

%blockworld
schema(bwMoveOn(X,gnd),[
	[bwPick(X),0,[X.free]],
	[bwPut(X,gnd),0,[hand.X]] ],
	[X.on.gnd],
	[] ).

schema(bwMoveOn(X,Y),[
	[bwPick(X),0,[X.free]],
	[bwPut(X,Y),0,[Y.free,hand.X]] ],
	[X.on.Y],
	[] ).

schema(bwPick(X), [], [hand.X],[]).
schema(bwPut(X,Y), [], [X.on.Y],[]).
schema(bwArm,[],[],[]).
schema(bwStream,[],[],[]).
schema(bwPlan(_), [], [a.on.b,b.on.c],[]).
schema(bwFree(X), [], [X.free],[]).
schema(bwLeaves, [], [freehand],[]).
schema(blockWorld,[
	[bwStream,0,["TRUE"]],
	[bwArm,0,["TRUE"]] ],
	[],
	[] ).

% NB tutto il caricamento del sottoschema è gestito lato C 
%	anche se si potrebbe fare il PROLOG
schema(sat(_), [], X, []):-ground(X).


%laas-unina integration 
%	(pr2) are concrete commands for PR2 robot
%	(ab) are abstract air-bus actions
schema(airbus,[
	[airbusStream,0,["TRUE"]],
	[pr2Stream,0,["TRUE"]]
	%[pr2Stop,0,["TRUE"]]
	],
	[],
	[] ).

schema(foresee(_,S,G),[
	[emp(S),0,[reach(S,G)]] ],
	[],
	[] ).

schema(foresight,[],[],[]).

schema(hatpStream,[],[],[]).

schema(pr2Stream,[],[],[]).
schema(pr2Point(_,_),[],[],[]).
schema(pr2Give(_,X), [], [X.onHuman],[]).
schema(pr2Receive(_,X), [], [X.onKit],[]).
schema(pr2WaitTime(X,Y), [], [pr2WaitTime(X,Y).done],[]).
schema(pr2Stop,[],[],[]).
schema(intention(_,_),[],[],[]).

schema(abInstall(_,_),[],[],[]).

schema(airbusStream,[
	[abInstall(slot160,[abClean(slot160),abGlue(slot160),abPlaceBracket(slot160)]),0,["TRUE"]]
	%[abInstall(slot480,[abClean(slot480),abGlue(slot480),abPlaceBracket(slot480)]),0,["TRUE"]]
	],
	[],
	[] ).

schema(airbusStreamOld,[
	[abInstall(slot160,[abClean(slot160),abApply(slot160),pr2WaitTime(slot160,30),abGlue(slot160),abPlaceBracket(slot160)]),0,["TRUE"]],
	[abInstall(slot480,[abClean(slot480),abApply(slot480),pr2WaitTime(slot480,30),abGlue(slot480),abPlaceBracket(slot480)]),0,["TRUE"]]
	],
	[],
	[] ).

schema(waitApply(_),[[pr2Stop,0,["TRUE"]]], [],[]).

schema(pointApply(Slot),[[pr2Point(Slot,apply),0,["TRUE"]] ],
	[intention(receive(cleanbottle),Slot)],
	[] ).
schema(pointGlue(Slot),[[pr2Point(Slot,glue),0,["TRUE"]] ],
	[intention(receive(glue),Slot)],
	[] ).

schema(abClean(Slot),[
	[pr2Point(Slot,clean),0,["TRUE"]] ],
	[intention(give(glue),Slot)],[]).

schema(abApply(Slot),[
	[pr2Give(Slot,cleanbottle),0,[intention(give(cleanbottle),Slot)]],
	[pointApply(Slot),0,[cleanbottle.onHuman]],
	[pr2Receive(Slot,cleanbottle),0,[intention(receive(cleanbottle),Slot),cleanbottle.onHuman]] ],
	[intention(receive(cleanbottle),Slot),cleanbottle.onKit],
	[] ).

schema(abGlue(Slot),[
	[pr2Give(Slot,glue),0,[glue.onKit]],
	[pointGlue(Slot),0,[glue.onHuman]],
	[pr2Receive(Slot,glue),0,[intention(receive(glue),Slot),glue.onHuman]] ],
	[intention(receive(glue),Slot),glue.onKit],
	[] ).

schema(abPlaceBracket(Slot),[
	[pr2Give(Slot,plate),0,["TRUE"]],
	[pr2Point(Slot,bracket),0,[plate.onHuman]] ],
	[intention(next,Slot)],
	[] ).

%SHERPA INTERFACE
schema(drone(_),[],[],[]).
schema(sherpa,[
	[drone(redDrone),0,["TRUE"]],
	[drone(blueDrone),0,["TRUE"]],
	[drone(whiteDrone),0,["TRUE"]],
	[drone(yellowDrone),0,["TRUE"]],
	[drone(cianoDrone),0,["TRUE"]],
	%[drone(violetDrone),0,["TRUE"]],
	[drone(orangeDrone),0,["TRUE"]],
	[drone(brownDrone),0,["TRUE"]],
	%[drone(pinkDrone),0,["TRUE"]],
	[drone(greenDrone),0,["TRUE"]]],
	[],
	[] ).
schema(sf,[],[],[]).

schema(resque(X),[
	[takeoff(X),0,[X.takeoff]],
	[reaching(X),0,[X.reaching]],
	[searching(X),0,[X.searching]],
	[inspecting(X),2,[X.inspecting]],
	[landing(X),0,[X.landing]]],
	[],
	[] ).

schema(takeoff(_),[],[],[]).
schema(reaching(_),[],[],[]).
schema(searching(X),[drone(X)],[],[]).
schema(inspecting(X),[drone(X),showcamera(X)],[],[]).
schema(landing(_),[],[],[]).

schema(showcamera(_),[],[],[]).


% Kinesthetic Teaching

%schema(kuka(_),[],[]).
%schema(kuka(_,_),[],[]).
%schema(kukaExecute(_,_),[],[]).
%schema(subtask(_,_),[],[]).

schema(ktStream,[],[],[]).

schema(kuka(_),[],[],[]). %TO BE RENAMED
schema(kuka(_,_),[],[],[]). %TO BE RENAMED

schema(dmp(_,_),[],[],[]).

%-- prepare tea and prepare coffee scenario
schema(preparecoffee,[
	%[add(milk),0,["TRUE"]],
	[add(water),0,["TRUE"]],
	[add(coffee),0,["TRUE"]],
	[use(spoon),0,[water.used,coffee.used]] ],
	[spoon.used],
	[] ).

schema(preparetea,[
	%[add(milk),0,["TRUE"]],
	[add(water),0,["TRUE"]],
	%[add(coffee),0,["TRUE"]],
	[add(tea),0,[water.used]] ],
	[tea.used],
	[] ).

schema(add(OBJ),[
	[subtask(take,OBJ),0,[hand.free]],
	[subtask(poor,OBJ),0,[OBJ.taken]] ],
	[OBJ.used],
	[] ).

schema(use(OBJ),[
	[subtask(take,OBJ),0,[hand.free]],
	[subtask(mix,OBJ),0,[OBJ.taken]] ],
	[OBJ.used],
	[] ).

schema(subtask(take,Obj),[],[Obj.taken],[]).
schema(subtask(poor,Obj),[],[Obj.used],[]).
schema(subtask(mix,Obj),[],[Obj.used],[]).
%--


schema(subtask(_,_),[],[],[]). %is this needed?



%blockworld (il goal è sempre tra gli effetti)
effect(bwMoveOn(X,Y),[X.on.Y,-Y.free],[]).
effect(bwPick(X),[hand.X,-X.free],[]).
effect(bwPut(X,gnd),[-hand.X,X.free],[]).
effect(bwPut(X,Y),[X.on.Y,-hand.X,-Y.free,X.free],[]).
effect(bwFree(X),[X.free],[]).
effect(bwPlan(_),[a.on.b,b.on.c],[]).

result(S,R):-goal(S,G),effect(S,E),append(G,E,R).
result(S,R):-goal(S,R).
result(S,R):-effect(S,R).

%blockworld
constraint(bwPick(X),[X.free,freehand],[]).
constraint(bwPut(X,gnd),[hand.X],[]).
constraint(bwPut(X,Y),[hand.X,Y.free],[]).
constraint(bwFree(_),[freehand],[]).
constraint(bwLeaves,[-freehand],[]).

constraint(reachColor(C),[C.present],[]).
constraint(place(Obj,_,ground),[knapsack.Obj],[]).

%effect(bwPut(X,Y),[hand.free,-hand.X,-Y.free],[]).




%%% %%% %%% Kinesthetic Teaching (update of the LTM) %%% %%% %%%

updateSchema(Instance,NewSon,Releaser,null):-
		retract(schema(Instance,[H|Rest],Goal)),!,
		append([H|Rest],[[NewSon,0,[Releaser]]],NewList),
		asserta(schema(Instance,NewList,Goal)).
		%listing(schema/2).

updateSchema(Instance,NewSon,Releaser,Goal):-
		retract(schema(Instance,[H|Rest],OldGoal)),!,
		append([H|Rest],[[NewSon,0,[Releaser]]],NewList),
		asserta(schema(Instance,NewList,OldGoal)),
		assertz(goal(NewSon,[Goal])). %%[TODO] what does this mean?
		%listing(schema/2).

updateSchema(Instance,NewSon,Releaser,Goal):-!,
		asserta(schema(Instance,[[NewSon,0,[Releaser]]])),
		assertz(goal(NewSon,[Goal])).

%saveLTM:-tell('LTM_autosave.prolog'), listing(schema/2), told.
saveLTM:-
	open('LTM_autosave.prolog',write,S),
	set_stream(output,S),
	listing(schema/2),
	listing(goal/2),
	set_stream(output,stdout),
	%listing(schema/2),
	close(S).



%%% %%% %%% LTM utils %%% %%% %%%

%get the level of abstraction of a schema (ie. max_tree_depth-1)
absLevel(S,0):-schema(S,[]). %concrete
absLevel(S,N):-schema(S,L), maxAbsLevel(L,LN),N is LN+1.
%maxAbsLevel([],0).
maxAbsLevel([[SS,_,_]|Rest],RN):-maxAbsLevel(Rest,RN),absLevel(SS,SSN),RN>=SSN.
maxAbsLevel([[SS,_,_]|_],SSN):-absLevel(SS,SSN).

%get the list of sub-schemata SSL for the schema S (ie. discard releasers and rtms)
subSchemaList([[SS,_,_]|Rest],[SS|SSRest]):-subSchemaList(Rest,SSRest).
subSchemaList([[SS,_,_]],[SS]).
subSchemaList(S,SSL):-schema(S,List),subSchemaList(List,SSL).

%%% this seems useless:
%schemaInstance(S,L):-schema(S,L),instanceList(L).
%instanceList([]).
%instanceList([[S,_,_]|Rest]):-schemaInstance(S,_),instanceList(Rest).
