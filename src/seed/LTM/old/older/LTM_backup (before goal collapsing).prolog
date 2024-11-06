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
	schema(X,SemDef),
	yield(SemDef,_),!.

getGoal(X):-
	yield([],_),
	goal(X,Goal),
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

%schema: nome,[[subschema1(Param),importanza,[releaserList]], [subschema2...]...]

%schemi ASTRATTI
%schema(wanderUntil(X),[[wander,1,[X.asd]], [avoid,1,[t]]]).

schema(gui,[]).
schema(map,[]).

%LEARNING AVOIDANCE
schema(alive,[
	[sonarStreamLearn,1,["TRUE"]],
	[engineStream,1,["TRUE"]],
	[blobStream,1,["TRUE"]],
	[inputStream,1,["TRUE"]],
	[rosStream,1,["TRUE"]],
	%[show(alive),1,["TRUE"]],
	%[show(alive,less),1,["TRUE"]],
	[show(requestStream),1,["TRUE"]],
	%[gui,1,["TRUE"]],
	%[avoidLearn,1,["TRUE"]],
	[joyStream,1,["TRUE"]],
	[requestStream,1,["TRUE"]]]).

schema(q,[[forget(alive),1,["TRUE"]]]).

schema(avoidLearn,[
	[avoidObstacle,1,[sonar.obstacle]],
	[gotoxy(-6,6,asd),1,["TRUE"]]]).

schema(avoidObstacle,[
	[goRun,1,["TRUE"]],
	[goFast,1,["TRUE"]],
	[goSlow,1,["TRUE"]],
	[goNot,1,["TRUE"]],
	[goBack,1,["TRUE"]],
	[turnLeftAll,1,["TRUE"]],
	[turnLeft,1,["TRUE"]],
	[turnLeftFore,1,["TRUE"]],
	[turnNot,1,["TRUE"]],
	[turnRightAll,1,["TRUE"]],
	[turnRight,1,["TRUE"]],
	[turnRightFore,1,["TRUE"]]]).

schema(goRun,[]).
schema(goFast,[]).
schema(goSlow,[]).
schema(goNot,[]).
schema(goBack,[]).
schema(turnLeftAll,[]).
schema(turnLeft,[]).
schema(turnLeftFore,[]).
schema(turnNot,[]).
schema(turnRightAll,[]).
schema(turnRight,[]).
schema(turnRightFore,[]).

schema(learn2cross,[
	[crossRoad,1,["TRUE"]],
	[semaphore,1,["TRUE"]]
	]).

schema(crossRoad,[
	[waitSemaphore,1,["TRUE"]],
	[cross,1,["TRUE"]]
	]).

schema(waitSemaphore,[
	[stop,1,["TRUE"]]
	]).

schema(stop,[
	[turnNot,1,["TRUE"]],
	[goNot,1,["TRUE"]]
	]).

schema(cross,[
	[gotoxy(0,5,crossRoad),1,[sempahore1.arrived]],
	[gotoxy(0,-5,crossRoad),1,[sempahore2.arrived]],
	[avoid,1,["TRUE"]]
	]).

schema(semaphore,[]).

schema(wanderUntilCollide,[
	[wander,1,["TRUE"]],
	[forget(wanderUntilCollide),1,[engine.collide]]]).

schema(avoidWalk,[
	[wander,1,["TRUE"]],
	[avoid,1,["TRUE"]]]).

schema(goto(X,Y,ID),[
	[avoid,1,["TRUE"]],
	[gotoxy(X,Y,ID),1,["TRUE"]]]).

schema(followColor(Color),[
	[avoid,2,["TRUE"]],
	[reachColor(Color),1,["TRUE"]]]).

schema(searchColorRandomly(Color),[
	[avoid,1,[-Color.near]],
	[wander,1,[-Color.present]],
	[reachColor(Color),1,[Color.present]]]).

schema(explorexy(X,Y,ID),[
	[avoid,1,["TRUE"]],
	[gotoxy(X,Y,ID),1,["TRUE"]],
	[wander,1,[gotoxy(X,Y,ID).arrived]]]).

schema(gotoColor(X,Y,Color),[
	[explorexy(X,Y,gotoColor(X,Y,Color)),1,[-Color.present]],
	[followColor(Color),1,[Color.present]]]).

schema(pickup(Obj),[
	[place(Obj,C,knapsack),1,[-knapsack.full,C.reached]]]):-color(Obj,C).

schema(putdown(Obj),[
	[place(Obj,C,ground),1,[knapsack.Obj]]]):-color(Obj,C).

schema(take(Obj,X,Y),[
	[gotoColor(X,Y,Color),1,[-knapsack.full,-knapsack.Obj,-Color.reached]],
	[pickup(Obj),1,[Color.reached,-knapsack.full,-knapsack.Obj]]]):-color(Obj,Color).
	
schema(takeToMe(Obj),[
	[takeObject(Obj),1,["TRUE"]],
	[return(Obj),1,["TRUE"]]]).

schema(takeObject(Obj),[
	[gotoObject(Obj),1,[-knapsack.full,-knapsack.Obj,-Color.reached]],
	[pickup(Obj),1,[Color.reached,-knapsack.full,-knapsack.Obj]]]):-color(Obj,Color).

schema(gotoObject(Obj),[
	[explorexy(X,Y,Obj),1,[-Color.present]],
	[followColor(Color),1,[Color.present]]]):-color(Obj,Color),position(Obj,Land),landmark(Land,X,Y).

schema(return(Obj),[
	[gotoObject(worker),1,[knapsack.Obj]],
	[putdown(Obj),1,[blue.reached]]]).

%questi schemi sono per ICSR
schema(take4,[
	[takeToMe(objRed),1,["TRUE"]],
	[takeToMe(objGreen),1,["TRUE"]],
	[takeToMe(objBrown),1,["TRUE"]],
	[takeToMe(objYellow),1,["TRUE"]]]).

schema(take3,[
	[takeToMe(objRed),1,["TRUE"]],
	[takeToMe(objGreen),1,["TRUE"]],
	[takeToMe(objYellow),1,["TRUE"]]]).

schema(take2,[
	[takeToMe(objRed),1,["TRUE"]],
	[takeToMe(objGreen),1,["TRUE"]]
	]).

%sequential veresion
schema(takeNseq,$[
	[takeToMe(objRed),1,["TRUE"]],
	[takeToMe(objYellow),1,["TRUE"]],
	[takeToMe(objBrown),1,["TRUE"]],
	[takeToMe(objGreen),1,["TRUE"]]
	]).

schema(takeN,[
	[takeToMe(objRed),1,["TRUE"]],
	[takeToMe(objYellow),1,["TRUE"]],
	[takeToMe(objOrange),1,["TRUE"]],
	[takeToMe(objBrown),1,["TRUE"]],
	[takeToMe(objPink),1,["TRUE"]],
	[takeToMe(objGreen),1,["TRUE"]]
	]).

schema(take1(X),[
	[taketo(X,worker),1,["TRUE"]]]).

schema(taketo(Obj,Pos),[
	[take(Obj,Xobj,Yobj),1,[-knapsack.Obj]],
	[gotoColor(Xpos,Ypos,PosColor),1,[knapsack.Obj]],
	[putdown(Obj),1,[knapsack.Obj,PosColor.reached]]]):-
		color(Pos,PosColor),
		position(Obj,Xobj,Yobj),
		position(Pos,Xpos,Ypos).

schema(routetoColor(Subject),[
	[routeto(Subject),1,[-Color.present]],
	[followColor(Color),1,[Color.present]]]):-color(Subject,Color).

schema(plantake(Obj,Pos),[
	[routetoColor(Obj),1,[-ObjColor.reached,-knapsack.full,-knapsack.Obj]],
	[pickup(Obj),1,[ObjColor.reached,-knapsack.full,-knapsack.Obj]],
	[routetoColor(Pos),1,[-PosColor.reached,knapsack.Obj]],
	[putdown(Obj),1,[knapsack.Obj,PosColor.reached]]]):-
		color(Pos,PosColor),
		color(Obj,ObjColor).

schema(gotopoi(X,Y),[
	[poi(X,Y),1,["TRUE"]],
	[avoid,1,["TRUE"]]]).


%-- tests for speech recognition
schema(go\to\position\Px\Py,[
	[reachpos(Px,Py,0),1,["TRUE"]] ]).

schema(hello,[
	[say("hello"),1,["TRUE"]] ]).

schema(go\to\position\Px\Py\Pw,[
	[reachpos(Px,Py,Pw),1,["TRUE"]] ]).
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

schema(deliver\kit\K,[
	[prepare\kit\K,1,["TRUE"]],
	[goto(land_meeting),1,[K\is\ready]] ]).

%kit k1
schema(prepare\K,[
	[add\component\gear\from\s1\to\K,1,["TRUE"]],
	[add\component\tube\from\s2\to\K,1,["TRUE"]],
	[add\component\bar\from\s2\to\K,1,["TRUE"]],
	[add\component\screw\from\s3\to\K,1,["TRUE"]] ]).

%kit k2

schema(add\component\C\from\S\to\K,[
	[reach\component\C\on\shelf\S,1,["TRUE"]],
	[pick\component\C\from\S,1,[C\is\reached]],
	[place\component\C\on\K,1,[C\is\taken]] ]).

schema(reach\component\C\on\shelf\S,[
	[move\to\S,1,["TRUE"]],
	[move\to\C,1,[pose\of\C\is\known]] ]).

%schema(move\to\shelf\S,[
%	[avoid,1,["TRUE"]],
%	[goto(L),1,["TRUE"]] ]):-pose(S,L).

%schema(move\to\component\C,[
%	[goto(L),1,"TRUE"] ]):-pose(C,L).

schema(move\to\landmark\L,[
	[avoid,1,["TRUE"]],
	[goto(L),1,["TRUE"]] ]):-pose(_,L).

schema(move\to\X,[
	[avoid,1,["TRUE"]],
	[goto(X),1,["TRUE"]] ]).

schema(pick\component\C\from\_,[
	[moveArm(C),1,[gripper\is\free]],
	[closeGripper,1,[arm\is\in\pose\C]] ]).

schema(place\component\C\on\kit\K,[
	[moveArm(K),1,[C\is\taken]],
	[openGripper,1,[arm\is\in\pose\K]] ]).

%open-close gripper
schema(closeGripper,[]).
schema(openGripper,[]).

%move the arm to a pose
schema(moveArm(_),[]).
%move the robot to a landmark
schema(goto(_),[]).
%avoid obstacles (mobile)
%schema(avoid,[]). %already exists


%ICOSAF - end




schema(reachpos(Px,Py,Pw),[
	[laserStream,1,["TRUE"]],
	[avoid,1,["TRUE"]],	
	[goinpos(Px,Py,Pw),1,["TRUE"]] ]).

schema(conflict,[
	[take(objRed,-2,0),1,[-place(objRed,red,ground).done]],
	[take(objGreen,2,0),1,[-place(objGreen,green,ground).done]]]).

schema(conflict2,[
	[take(objRed,-2,0),1,[-place(objRed,red,ground).done]],
	[gotoColor(6,6,blue),1,[knapsack.objRed]],
	[putdown(objRed),1,[knapsack.objRed,blue.reached]],
	[take(objGreen,2,0),1,[-place(objGreen,green,ground).done]],
	[gotoColor(6,6,blue),1,[knapsack.objGreen]],
	[putdown(objGreen),1,[knapsack.objGreen,blue.reached]]]).

schema(request,[[takeObject(objGreen),1,["TRUE"]],
	[takeObject(objRed),3,["TRUE"]] ]).


%sequential speak test
schema(sayseq,$[
	[say("this is"),1,["TRUE"]],
	[say("a test"),1,["TRUE"]],
	[say("of sequential"),1,["TRUE"]],
	[say("speech."),1,["TRUE"]]
	]).

%longer sequence
schema(ahab,$[
	[say("God help thee, old man, thy thoughts have created a creature in thee."),1,["TRUE"]],
	[say("and he, whose intense thinking thus makes him a Prometheus,"),1,["TRUE"]],
	[say("a vulture feeds upon that heart forever."),1,["TRUE"]],
	[say("the vulture, the very creature he creates."),1,["TRUE"]]
	]).

%schemi CONCRETI (ie. behavior)
schema(wander,[]).
schema(forget(_),[]).
schema(sonarStream,[]).
schema(engineStream,[]).
schema(blobStream,[]).
schema(laserStream,[]).
schema(inputStream,[]).
schema(remember(_),[]).
schema(listing,[]).
schema(avoid,[]).
schema(gotoxy(_,_,_),[]).
schema(goinpos(_,_,_),[]).
%schema(reachColor(_),[]).
schema(reachColor(C),[]):-color(_,C).
schema(place(_,_,_),[]).
schema(fg(_),[]).
schema(emp(_),[]).
schema(show(_),[]).
schema(show(_,_),[]). %is the old showLess
schema(wait,[]).
schema(requestStream,[]).
schema(routeto(_),[]).
schema(test,[]).
schema(rosStream,[]).
schema(joyStream,[]).


schema(ltm(_),[]).
schema(set(_,_,_),[]).
schema(set(_,_),[]). %means only once
schema(send(_,_,_,_),[]).
schema(send(_,_,_),[]). %means only once

schema(decide(_,_),[]).
schema(freeze(_),[]).
schema(audioStream,[[juliusAudioRecognizer,1,["TRUE"]]]).
%schema(audioStream,[[googleAudioRecognizer,1,["TRUE"]]]).
schema(googleAudioRecognizer,[]).
schema(juliusAudioRecognizer,[]).
schema(vocalStream,[]).
schema(say(_),[]).
schema(dialogue(_,_),[]).

schema(exePlan(_),[]).
schema(exeSeq(_),[]).
schema(setPlayer,[]).
schema(poiStream,[]).
schema(poi(_,_),[]).
schema(emp(_),[]).
schema(cpuMonitor,[]).
schema(gui,[]).

schema(randsetPlayer,[]).

schema(sonarStreamLearn,[]).
schema(send(_,_),[]).

schema(tmPlanner,[]).

%SAPHARI Airbus Plan-Exe (ape)
schema(ape,[
	[exePlan([search(bracket1),take(bracket1),give(bracket1,worker)]),1,["TRUE"]],
	[installBracket(bracket1,location1),1,["TRUE"]],
	[show(installBracket(bracket1,location1)),1,["TRUE"]]
	]).

schema(installBracket(Obj,Pos),[
	%[receive(Obj,C),1,[worker.Obj]],
	[get(Obj),1,[-worker.Obj,-knapsack.full]],
	[give(Obj,worker),1,[knapsack.Obj]],
	[attachBracket(Obj,Pos),1,[knapsack.Obj]]
	]).%:-color(Obj,C).

schema(get(Obj),[
	[take(Obj),1,[-knapsack.full]],
	[searchColor(Color),1,[-knapsack.full,-Color.present]]
	]):-color(Obj,Color).

schema(searchColor(_),[]).

schema(search(Obj),[
	[searchColor(Color),1,["TRUE"]]
	]):-color(Obj,Color).

%schema(search(Obj),[
%	[goto(X1,Y1,search(Obj)),1,["TRUE"]],
%	[goto(X2,Y2,search(Obj)),1,["TRUE"]]
%	]):-landmark(worker,X1,Y1),landmark(yellow,X2,Y2).

schema(take(Obj),[
	[followColor(Color),1,[-knapsack.full,-knapsack.Obj,-Color.reached]],
	[pickup(Obj),1,[Color.reached,-knapsack.full,-knapsack.Obj]]
	]):-color(Obj,Color).

schema(give(Obj,worker),[
	[goto(0,0,give(Obj,worker)),1,[-blue.present,knapsack.Obj,-gotoxy(0,0,give(Obj,worker)).near]],
	[handover(Obj,worker),1,["TRUE"]]
	%[bring(Obj,location1),1,["TRUE"]],
	%[bring(Obj,location2),1,["TRUE"]],
	%[bring(Obj,location3),1,["TRUE"]]
	]).%:-landmark(center,X,Y).

schema(attachBracket(Obj,Pos),[
	[bring(Obj,Pos),1,["TRUE"]]
	]).

schema(bring(Obj,Pos),[
	[gotoObject(Pos),1,[knapsack.Obj]],
	[putdown(Obj,Pos),1,[C.reached]]
	]):-color(Pos,C).

schema(handover(Obj,worker),[
	[followColor(blue),1,[knapsack.Obj]],
	[putdown(Obj,worker),1,[blue.reached]]
	]).

schema(putdown(Obj,Pos),[
	[place(Obj,C,Pos),1,[knapsack.Obj]]
	]):-color(Pos,C).

schema(receive(_,_),[]).


%RO-MAN
schema(simuarm,[]).
schema(simutake(_),[]).
schema(makeCoffee,[[simutake(coffee),1,["TRUE"]],
	[simutake(cup),1,["TRUE"]],
	[simutake(sugar),1,["TRUE"]],
	[simutake(spoon),1,["TRUE"]]]).

schema(makeTea,[[simutake(tea),1,["TRUE"]],
	[simutake(cup),1,["TRUE"]],
	[simutake(sugar),1,["TRUE"]],
	[simutake(spoon),1,["TRUE"]]]).

schema(roman,[[simuarm,1,["TRUE"]],
        [makeCoffee,1,[-tea.taked]],
        [makeTea,1,[-coffee.taked]],%]).
	[say(en("done, have a good coffee!")),1,[coffee.taked,cup.taked,spoon.taked,sugar.taked]],
	[say(en("done, have a good tea!")),1,[tea.taked,cup.taked,spoon.taked,sugar.taked]]]).


%elaborazione del linguaggio IT


schema(it("chiudi"),[[forget(alive),1,["TRUE"]]]).
schema(it("lista"),[[listing,1,["TRUE"]]]).

schema(note,[[amplify(simutake(coffee),-1),1,["TRUE"]]]).
schema(nocaffe,[[amplify(simutake(tea),-1),1,["TRUE"]]]).


schema(it(Sentence),[]):-
	string(Sentence),
	split_string(Sentence," ","",[_|[]]).

schema(it(Sentence),[
	[it(Word),1,["TRUE"]],
	[it(SubSentence),1,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[Word|Rest]),
		join_string(Rest," ",SubSentence).

schema(saySequence(it(Sentence)),[[say(it(Sentence)),1,["TRUE"]]]):-
	string(Sentence),
	split_string(Sentence," ","",[_|[]]).

schema(saySequence(it(Sentence)),[
	[say(it(Word)),1,["TRUE"]],
	[saySequence(it(SubSentence)),1,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[Word|Rest]),
		join_string(Rest," ",SubSentence).

%schema(segment(Seg),[]):-!,is_list(Seg).

schema(saySegment,[[segment([say(it("sopra")),say(it("la")),say(it("panca")),say(it("la")),say(it("capra")),say(it("campa"))]),1,["TRUE"]]]).

schema(segment([Head|[]]),[[Head,1,["TRUE"]]]).

schema(segment([Head|Rest]),[
	[Head,1,["TRUE"]],
	[segment(Rest),1,HeadGoal]]):-
		goal(Head,HeadGoal).

schema(segment([Head|Rest]),[
	[Head,1,["TRUE"]],
	[segment(Rest),1,["TRUE"]]]).

schema(amplify(S,_),[]):-
		schema(S,_).

schema(adm,[]).
schema(hai,[]).


%schemi SAPHARI

schema(saphari,[
	[gestureRecognition,1,["TRUE"]],
	[objectRecognition,1,["TRUE"]],
	%[audioStream,1,["TRUE"]],
	[fusionEngine,1,["TRUE"]]]).

schema(gesture(take),[]).
schema(gesture(give),[]).
schema(gesture(stop),[]).
schema(gesture(no),[]).
schema(gesture(leaves),[]).
schema(gesture(point),[]).
schema(gesture(come),[]).
schema(gesture(search),[]).
%schema(gesture(pointleft),[]).
%schema(gesture(pointfront),[]).

schema(speech(take),[]).
schema(speech(give),[]).
schema(speech(stop),[]).
schema(speech(no),[]).
schema(speech(leaves),[]).
schema(speech(come),[]).
schema(speech(search),[]).

schema(speech(next),[]).
schema(speech(done),[]).
schema(speech(ok),[]).


schema(object(cleanbottle),[]).

schema(object(cloth),[]).
schema(object(screw),[]).
schema(object(screwdriver),[]).
schema(object(plate),[]).
schema(object(glue),[]).
schema(object(null),[]).

schema(vrepExec(_,_),[]).

schema(fusionEngine,[]).

schema(gestureRecognition,[]).
%schema(gestureRecognition,[
%	[gesture(point),0,["TRUE"]],
%	[gesture(take),0,["TRUE"]],
%	[gesture(come),0,["TRUE"]],
%	[gesture(leaves),0,["TRUE"]],
%	[gesture(no),0,["TRUE"]],
%	[gesture(give),0,["TRUE"]],
%	[gesture(search),0,["TRUE"]],
%	[gesture(stop),0,["TRUE"]]]).

schema(objectRecognition,[
	[object(null),1.5,["TRUE"]]]).
%schema(objectRecognition,[
%	[object(cloth),0,["TRUE"]],
%	[object(screw),0,["TRUE"]],
%	[object(plate),0,["TRUE"]],
%	[object(glue),0,["TRUE"]],
%	[object(screwdriver),0,["TRUE"]]]).

%elaborazione del linguaggio per SAPHARI
schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,Noun,Object]),
		term_string(TermAction,Action),
		term_string(me,Noun),
		term_string(TermObject,Object).

	%oggetti composti (eg. screwdriver)
schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,"me",Ob_pt1,Ob_pt2]),
		term_string(TermAction,Action),
		concat_string([Ob_pt1,Ob_pt2],Object),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,Ob_pt1,Ob_pt2]),
		term_string(TermAction,Action),
		concat_string([Ob_pt1,Ob_pt2],Object),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]],
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action,Object]),
		term_string(TermAction,Action),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[speech(TermAction),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Action]),
		term_string(TermAction,Action),
		schema(speech(TermAction),_).

% i prossimi 2 sono aggiunti prima di tolosa
schema(en(Sentence),[
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Ob_pt1,Ob_pt2]),
		concat_string([Ob_pt1,Ob_pt2],Object),
		term_string(TermObject,Object).

schema(en(Sentence),[
	[object(TermObject),2,["TRUE"]]]):-
		string(Sentence),
		split_string(Sentence," ","",[_,Object]),
		term_string(TermObject,Object).

%blockworld
schema(bwMoveOn(X,gnd),[
	[bwPick(X),1,[X.free]],
	[bwPut(X,gnd),1,[hand.X]]]).

schema(bwMoveOn(X,Y),[
	[bwPick(X),1,[X.free]],
	[bwPut(X,Y),1,[Y.free,hand.X]]]).

schema(bwPick(_),[]).
schema(bwPut(_,_),[]).
schema(bwArm,[]).
schema(bwStream,[]).
schema(bwPlan(_),[]).
schema(bwFree(_),[]).
schema(bwLeaves,[]).
schema(blockWorld,[
	[bwStream,1,["TRUE"]],
	[bwArm,1,["TRUE"]]]).

% NB tutto il caricamento del sottoschema è gestito lato C 
%	anche se si potrebbe fare il PROLOG
schema(sat(_),[]).


%laas-unina integration 
%	(pr2) are concrete commands for PR2 robot
%	(ab) are abstract air-bus actions
schema(airbus,[
	[airbusStream,1,["TRUE"]],
	[pr2Stream,1,["TRUE"]]
	%[pr2Stop,1,["TRUE"]]
	]).

schema(foresee(_,S,G),[
	[emp(S),1,[reach(S,G)]]
	]).

schema(foresight,[]).

schema(hatpStream,[]).

schema(pr2Stream,[]).
schema(pr2Point(_,_),[]).
schema(pr2Give(_,_),[]).
schema(pr2Receive(_,_),[]).
schema(pr2WaitTime(_,_),[]).
schema(pr2Stop,[]).
schema(intention(_,_),[]).

schema(abInstall(_,_),[]).

schema(airbusStream,[
	[abInstall(slot160,[abClean(slot160),abGlue(slot160),abPlaceBracket(slot160)]),1,["TRUE"]]
	%[abInstall(slot480,[abClean(slot480),abGlue(slot480),abPlaceBracket(slot480)]),1,["TRUE"]]
	]).

schema(airbusStreamOld,[
	[abInstall(slot160,[abClean(slot160),abApply(slot160),pr2WaitTime(slot160,30),abGlue(slot160),abPlaceBracket(slot160)]),1,["TRUE"]],
	[abInstall(slot480,[abClean(slot480),abApply(slot480),pr2WaitTime(slot480,30),abGlue(slot480),abPlaceBracket(slot480)]),1,["TRUE"]]
	]).

schema(waitApply(_),[[pr2Stop,1,["TRUE"]]]).
schema(pointApply(Slot),[[pr2Point(Slot,apply),1,["TRUE"]]]).
schema(pointGlue(Slot),[[pr2Point(Slot,glue),1,["TRUE"]]]).

schema(abClean(Slot),[
	[pr2Point(Slot,clean),1,["TRUE"]]]).

schema(abApply(Slot),[
	[pr2Give(Slot,cleanbottle),1,[intention(give(cleanbottle),Slot)]],
	[pointApply(Slot),1,[cleanbottle.onHuman]],
	[pr2Receive(Slot,cleanbottle),1,[intention(receive(cleanbottle),Slot),cleanbottle.onHuman]]]).

schema(abGlue(Slot),[
	[pr2Give(Slot,glue),1,[glue.onKit]],
	[pointGlue(Slot),1,[glue.onHuman]],
	[pr2Receive(Slot,glue),1,[intention(receive(glue),Slot),glue.onHuman]]]).

schema(abPlaceBracket(Slot),[
	[pr2Give(Slot,plate),1,["TRUE"]],
	[pr2Point(Slot,bracket),1,[plate.onHuman]]]).

%SHERPA INTERFACE
schema(drone(_),[]).
schema(sherpa,[
	[drone(redDrone),1,["TRUE"]],
	[drone(blueDrone),1,["TRUE"]],
	[drone(whiteDrone),1,["TRUE"]],
	[drone(yellowDrone),1,["TRUE"]],
	[drone(cianoDrone),1,["TRUE"]],
	%[drone(violetDrone),1,["TRUE"]],
	[drone(orangeDrone),1,["TRUE"]],
	[drone(brownDrone),1,["TRUE"]],
	%[drone(pinkDrone),1,["TRUE"]],
	[drone(greenDrone),1,["TRUE"]]]).
schema(sf,[]).

schema(resque(X),[
	[takeoff(X),1,[X.takeoff]],
	[reaching(X),1,[X.reaching]],
	[searching(X),1,[X.searching]],
	[inspecting(X),2,[X.inspecting]],
	[landing(X),1,[X.landing]]]).

schema(takeoff(_),[]).
schema(reaching(_),[]).
schema(searching(X),[drone(X)]).
schema(inspecting(X),[drone(X),showcamera(X)]).
schema(landing(_),[]).

schema(showcamera(_),[]).

%goal laas-unina integration

goal(pointApply(Slot),[intention(receive(cleanbottle),Slot)]).
goal(pointGlue(Slot),[intention(receive(glue),Slot)]).

%goal(abClean(Slot),[intention(next,Slot)]).
goal(abClean(Slot),[intention(give(glue),Slot)]).
%goal(abClean(Slot),[intention(give(cleanbottle),Slot)]).
goal(abApply(Slot),[intention(receive(cleanbottle),Slot),cleanbottle.onKit]).
goal(abGlue(Slot),[intention(receive(glue),Slot),glue.onKit]).
goal(abPlaceBracket(Slot),[intention(next,Slot)]).
%goal(pr2Point(Slot),[intention(next,Slot)]).
goal(pr2WaitTime(X,Y),[pr2WaitTime(X,Y).done]).
goal(pr2Give(_,X),[X.onHuman]).
goal(pr2Receive(_,X),[X.onKit]).



%ICOSAF goal
goal(add\component\C\from\_\to\K,[C\is\placed\in\K]).
goal(reach\component\C\on\shelf\_,[C\is\reached]).
goal(move\to\landmark\L,[L\is\reached]).
goal(move\to\X,[X\is\reached]).
%goal(move\to\shelf\S,[S\is\reached]).
%goal(move\to\component\C,[C\is\reached]).
goal(pick\component\C\from\_,[C\is\taken]).
goal(place\component\C\on\K,[C\is\placed\in\K]).

goal(closeGripper,[gripper\is\close]).
goal(openGripper,[gripper\is\open]).

goal(moveArm(P),[P\is\reached]).
goal(goto(L),[L\is\reached]).
%ICOSAF - end



%goal degli schemi teleologici
goal(gotoxy(X,Y,ID),[gotoxy(X,Y,ID).arrived]).
goal(goto(X,Y,ID),[gotoxy(X,Y,ID).arrived]).
goal(reachColor(C),[C.reached]).
goal(gotoColor(_,_,C),[C.reached]).
goal(place(Obj,_,knapsack),[knapsack.Obj]).
goal(place(Obj,Col,Pos),[place(Obj,Col,Pos).done]).
goal(pickup(Obj),[knapsack.Obj]).
%goal(pickup(Obj),[place(Obj,ObjColor,knapsack).done]):-color(Obj,ObjColor).
goal(putdown(Obj),[place(Obj,ObjColor,ground).done]):-color(Obj,ObjColor).
goal(take(Obj,_,_),[knapsack.Obj]).
goal(taketo(Obj,_),[place(Obj,ObjColor,ground).done]):-color(Obj,ObjColor).
goal(plantake(Obj,Pos),[place(Obj,ObjColor,ground).done,PosColor.present]):-color(Pos,PosColor),color(Obj,ObjColor).
goal(routeto(Obj),[Color.present]):-color(Obj,Color).
goal(routetoColor(Obj),[Color.reached]):-color(Obj,Color).
goal(say(X),[say(X).done]).

goal(takeObject(Obj),[knapsack.Obj]).
goal(gotoObject(Obj),[C.reached]):-color(Obj,C).
goal(return(Obj),[place(Obj,ObjColor,ground).done]):-color(Obj,ObjColor).
goal(takeToMe(Obj),[place(Obj,ObjColor,ground).done]):-color(Obj,ObjColor).
goal(followColor(C),[C.reached]).

%goal(foresee(_,S,G),[reach(S,G)]).

%RO-MAN
goal(simutake(X),[X.taked]).

%blockworld
goal(bwMoveOn(X,Y),[X.on.Y]).
goal(bwPick(X),[hand.X]).
%goal(bwPut(_,gnd),[freehand]).
goal(bwPut(X,Y),[X.on.Y]).
goal(bwFree(X),[X.free]).
goal(bwPlan(_),[a.on.b,b.on.c]).
goal(bwLeaves,[freehand]).

% NB in X ci sono anche le variabili da non soddisfare!
goal(sat(X),X):-ground(X).

goal(poi(X,Y),[poi(X,Y).arrived]).
goal(gotopoi(X,Y),[poi(X,Y).arrived]).

%goal AIRBUS plan-exe
goal(installBracket(Obj,Pos),[Pos.Obj]).
goal(attachBracket(Obj,Pos),[Pos.Obj]).
goal(bring(Obj,Pos),[Pos.Obj]).
goal(handover(Obj,worker),[worker.Obj]).
goal(putdown(Obj,Pos),[place(Obj,C,Pos).done]):-color(Pos,C).
%goal(get(Obj),[knapsack.Obj]).
goal(get(Obj),[place(Obj,C,knapsack).done]):-color(Obj,C).
goal(search(Obj),[C.present]):-color(Obj,C).
goal(searchColor(C),[C.present]).
%goal(take(Obj),[knapsack.Obj]).
goal(take(Obj),[place(Obj,C,knapsack).done]):-color(Obj,C).
goal(give(Obj,X),[X.Obj]).

%blockworld (il goal è sempre tra gli effetti)
effect(bwMoveOn(X,Y),[X.on.Y,-Y.free]).
effect(bwPick(X),[hand.X,-X.free]).
effect(bwPut(X,gnd),[-hand.X,X.free]).
effect(bwPut(X,Y),[X.on.Y,-hand.X,-Y.free,X.free]).
effect(bwFree(X),[X.free]).
effect(bwPlan(_),[a.on.b,b.on.c]).

result(S,R):-goal(S,G),effect(S,E),append(G,E,R).
result(S,R):-goal(S,R).
result(S,R):-effect(S,R).

%blockworld
constraint(bwPick(X),[X.free,freehand]).
constraint(bwPut(X,gnd),[hand.X]).
constraint(bwPut(X,Y),[hand.X,Y.free]).
constraint(bwFree(_),[freehand]).
constraint(bwLeaves,[-freehand]).

constraint(reachColor(C),[C.present]).
constraint(place(Obj,_,ground),[knapsack.Obj]).

%effect(bwPut(X,Y),[hand.free,-hand.X,-Y.free]).



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
