%LTM taken from Riccard Grieco's thesis - here there is a ICOSAF schema list

:-op(500,xfy,.).
:-op(600,fx,-).
:-['Map2Memory.prolog'].

%SWI-prolog: in SWI the '.' operator is already used and constrained to take "dict" arguments 
%		..we have to replace it!
:-op(500,xfy,*.*).
.(L, R, L *.* R).

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
	%[sonarStreamLearn,1,["TRUE"]],
	%[engineStream,1,["TRUE"]],
	%[blobStream,1,["TRUE"]],
	[inputStream,1,["TRUE"]],
	[hcStream, 1, ["TRUE"]],
	[setOnce(set(hcStream.show, 0)), 1, ["TRUE"]], %TODO
	%[monitorWp, 1, ["TRUE"]],
	%[monitorWpNC, 1, ["TRUE"]],
	[rosStream,1,["TRUE"]],
	%[show(alive),1,["TRUE"]],
	%[show(alive,less),1,["TRUE"]],
	%[avoidLearn,1,["TRUE"]],
	%[joyStream,1,["TRUE"]],
	[requestStream,1,["TRUE"]]
	]).

schema(q,[[forget(alive),1,["TRUE"]]]).


%ICOSAF
schema(applyBigHead(_),[]).
schema(pickBigHead(_), []).
schema(getGlue, []).
schema(getGlue(_),[]).
schema(insertNutzer(_), []).
schema(tap(_), []).
schema(removeTool, []).
schema(putTool(_), []).
schema(report(_), []).
schema(reportWp(_, _, _),[]).

%schema(applyBigHeadTask(ID),[
%	[changeTool(glueDispenser),1,[-tool(glueDispenser)]],
%	[hcWaypoint(plate), 1, [tool(glueDispenser), -bigHead.picked]],
%	[pickBigHead(ID), 1, [waypoint(plate).reached]],
%	[hcWaypoint(dispenser), 1, [bigHead.picked, -glue.applied]],
%	[getGlue, 1 , [waypoint(dispenser).reached]],
%	[hcWaypoint(ID), 1, [glue.applied, -busy]],
%	[applyBigHead(ID), 1, [waypoint(ID).reached]]
%	]).

schema(approach(ID), [[hcWaypoint(ID.approach), 1, ["TRUE"]]]).

schema(applyGlue(ID), [
	%[hcWaypoint(ID), 1, ["TRUE"]], %automatic
	%[hcWaypoint(ID.circle), 1, [waypoint(ID).reached]],
	%[hcWaypoint(ID, true), 1, [tool(glueDispenser), -glue.applied, -busy]], %human guided
	%[dispenseGlue(ID), 1, [waypoint(ID).reached]]
	]).

schema(monitorGlue(ID), [
	%[set(busy, 1, 1, ID.glue), 1, [-glue(ID).applied, -busy, waypoint(ID.approach).reached]],
	%[setOnce(set(glue(ID).applied, 1, busy, 0, waypoint(ID.approach).reached, 0,
	% waypoint(ID).reached, 0, waypoint(ID.circle).reached, 0)), 1, [waypoint(ID.circle).reached, busy]]
	]).

schema(applyBigHeadTask(ID), [
	[changeTool(glueDispenser),1,[-tool(glueDispenser)]]%,
	%[approach(ID), 1, [tool(glueDispenser), -busy, -glue(ID).applied]],
	%[applyGlue(ID), 1, [tool(glueDispenser), waypoint(ID.approach).reached, busy]],
	%[monitorGlue(ID), 1, [tool(glueDispenser)]],
	%[applyBigHead(ID), 1, [glue(ID).applied]]
	]).

schema(insertNutzerTask(ID),[
	[changeTool(nutzerTool), 1, [-tool(nutzerTool)]],
	[hcWaypoint(ID), 1, [tool(nutzerTool), -busy]],
	[insertNutzer(ID), 1, [waypoint(ID).reached]]
	]).

schema(executeTapping(ID), [
	%[hcWaypoint(ID), 1, ["TRUE"]], %automatic
	%[hcWaypoint(ID.tap), 1, [waypoint(ID).reached, -halfTap(ID).done]],
	%[hcWaypoint(ID.back), 1, [waypoint(ID.tap).reached]],
	%[tap(ID), 1, [waypoint(ID).reached]]
	]).

schema(monitorTapping(ID), [
	%[set(busy, 1, 1, ID.tapping), 1, [waypoint(ID.approach).reached, -busy]],
	%[setOnce(set(halfTap(ID).done, 1)), 1, [waypoint(ID.tap).reached]],
	%[setOnce(set(tap(ID).done, 1, busy, 0)), 1, [halfTap(ID).done, -waypoint(ID.tap).reached]], %human takes away early
	%[setOnce(set(tap(ID).done, 1, busy, 0, waypoint(ID.approach).reached, 0,
	% waypoint(ID).reached, 0, waypoint(ID.back).reached, 0, waypoint(ID.tap).reached, 0)),1, [waypoint(ID.back).reached]]
	]).

schema(monitorTapping2(ID), [ %not-proactive
	[set(busy, 1, 1, ID.tapping), 1, [waypoint(ID.approach).reached, -busy]],
	[setOnce(set(halfTap(ID).done, 1)), 1, [waypoint(ID.tap).reached]],
	[setOnce(set(tap(ID).done, 1, busy, 0)), 1, [halfTap(ID).done, -waypoint(ID.tap).reached]] %human takes away early
	%[setOnce(set(tap(ID).done, 1, busy, 0, waypoint(ID.approach).reached, 0,
	 %waypoint(ID).reached, 0, waypoint(ID.back).reached, 0, waypoint(ID.tap).reached, 0)),1, [waypoint(ID.back).reached]]
	]).


schema(tappingTask(ID),[
	[changeTool(tap), 1, [-tool(tap)]]%,
	%%[hcWaypoint(ID, true), 1, [tool(tap), -busy]], %human guided
	%[approach(ID), 1, [tool(tap), -busy]],
	%[executeTapping(ID), 1, [tool(tap), waypoint(ID.approach).reached, busy]],
	%[monitorTapping(ID), 1, [tool(tap)]]
	]).

schema(tappingTask2(ID),[
	[changeTool(tap), 1, [-tool(tap)]],
	%[hcWaypoint(ID, true), 1, [tool(tap), -busy]], %human guided
	[approach(ID), 1, [tool(tap), -busy]],
	[executeTapping(ID), 1, [tool(tap), waypoint(ID.approach).reached, busy]],
	[monitorTapping2(ID), 1, [tool(tap)]]
	]).

schema(changeTool(ID), [
	%%[hcWaypoint(changeToolPos, true), 1, [-busy]], %human guided
	%[hcWaypoint(changeToolPos), 1, [-busy]], %automatic
	%[removeTool, 1, [waypoint(changeToolPos).reached]],
	%[putTool(ID), 1, [tool(none), waypoint(changeToolPos).reached]]
	]).


% TEST with plan %TODO goals
schema(testPlan1, [ %proactive
		[tappingTask(1), 1, ["TRUE"]],
		[tappingTask(2), 1, ["TRUE"]],
		[tappingTask(3), 1, ["TRUE"]],
		[tappingTask(4), 1, ["TRUE"]],
		[tappingTask(5), 1, ["TRUE"]]
	]).

schema(testPlan1_b, [ %not proactive
		[tappingTask2(1), 1, ["TRUE"]],
		[tappingTask2(2), 1, ["TRUE"]],
		[tappingTask2(3), 1, ["TRUE"]],
		[tappingTask2(4), 1, ["TRUE"]],
		[tappingTask2(5), 1, ["TRUE"]]
	]).

schema(test1A, [ %passive
	[setOnce(set(start, 0, tool(tap), 1, currentTool, tap, hcStream.ignore, 1)), 1, ["TRUE"]], %INIT
	[monitorWpNC, 1, ["TRUE"]],
	[report(test1A), 1, ["TRUE"]],
	[hcWaypoint(start), 1, [-start]],
	[setOnce(set(start, 1)), 1, [-start, waypoint(start).reached]],
	[testPlan1_b, 1, [start]],
	[setOnce(set(executeReport,1)), 1, [start]],
	[setOnce(set(executeReport, 0)), 1, [tap(1).done, tap(2).done,tap(3).done,tap(4).done,tap(5).done]],
	[show(test1A,less), 1, ["TRUE"]],
	[restNC, 1, [start]]
	]).

schema(test1B, [ %guided
	[setOnce(set(start, 0, tool(tap), 1, currentTool, tap, hcStream.ignore, 0)), 1, ["TRUE"]], %INIT
	[monitorWp, 1, ["TRUE"]],
	[report(test1B), 1, ["TRUE"]],
	[hcWaypoint(start), 1, [-start]],
	[setOnce(set(start, 1)), 1, [-start, waypoint(start).reached]],
	[testPlan1_b, 1, [start]],
	[setOnce(set(executeReport,1)), 1, [start]],
	[setOnce(set(executeReport, 0)), 1, [tap(1).done, tap(2).done,tap(3).done,tap(4).done,tap(5).done]],
	[show(test1B,less), 1, ["TRUE"]],
	[rest, 1, [start]]
	]).

schema(show_contrib, [
	[monitorWp, 1, ["TRUE"]],
	[hcStream, 1, ["TRUE"]],
	[hcWaypoint(1), 1, ["TRUE"]],
	[hcWaypoint(2), 1, ["TRUE"]],
	[hcWaypoint(3), 1, ["TRUE"]],
	[hcWaypoint(4), 1, ["TRUE"]],
	[hcWaypoint(5), 1, ["TRUE"]]
	]).

schema(test1C, [ %proactive
	[setOnce(set(start, 0, tool(tap), 1, currentTool, tap, hcStream.ignore, 0)), 1, ["TRUE"]], %INIT
	[monitorWp, 1, ["TRUE"]],
	[report(test1C), 1, ["TRUE"]],
	[hcWaypoint(start), 1, [-start]],
	[setOnce(set(start, 1)), 1, [-start, waypoint(start).reached]],
	[testPlan1, 1, [start]],
	[setOnce(set(executeReport,1)), 1, [start]],
	[setOnce(set(executeReport, 0)), 1, [tap(1).done, tap(2).done,tap(3).done,tap(4).done,tap(5).done]],
	[show(test1C,less), 1, ["TRUE"]],
	[rest, 1, [start]]
	]).

schema(test1Bprova, [ %guided
	[setOnce(set(start, 0, tool(tap), 1, currentTool, tap, hcStream.ignore, 0)), 1, ["TRUE"]], %INIT
	[monitorWp, 1, ["TRUE"]],
	[report(prova), 1, ["TRUE"]],
	[hcWaypoint(start), 1, [-start]],
	[setOnce(set(start, 1)), 1, [-start, waypoint(start).reached]],
	[testPlan1_b, 1, [start]],
	[setOnce(set(executeReport,1)), 1, [start]],
	[setOnce(set(executeReport, 0)), 1, [tap(1).done, tap(2).done,tap(3).done,tap(4).done,tap(5).done]],
	[show(test1Bprova,less), 1, ["TRUE"]],
	[rest, 1, [start]]
	]).

schema(test1Cprova, [ %proactive
	[setOnce(set(start, 0, tool(tap), 1, currentTool, tap, hcStream.ignore, 0)), 1, ["TRUE"]], %INIT
	[monitorWp, 1, ["TRUE"]],
	[report(test1Cprova), 1, ["TRUE"]],
	[hcWaypoint(start), 1, [-start]],
	[setOnce(set(start, 1)), 1, [-start, waypoint(start).reached]],
	[testPlan1, 1, [start]],
	[setOnce(set(executeReport,1)), 1, [start]],
	[setOnce(set(executeReport, 0)), 1, [tap(1).done, tap(2).done,tap(3).done,tap(4).done,tap(5).done]],
	[show(test1Cprova,less), 1, ["TRUE"]],
	[rest, 1, [start]]
	]).

%old
schema(testP1C, [
	[setOnce(set(start, 0, tool(tap), 1, currentTool, tap, hcStream.ignore, 0)), 1, ["TRUE"]], %INIT
	%[monitorWp, 1, ["TRUE"]],
	[report(testP1C), 1, ["TRUE"]],
	[hcWaypoint(start), 1, [-start]],
	[setOnce(set(start, 1)), 1, [-start, waypoint(start).reached]],
	[testPlan1, 1, [start]],
	[setOnce(set(executeReport,1)), 1, [start]],
	[setOnce(set(executeReport, 0)), 1, [tap(1).done, tap(2).done,tap(3).done,tap(4).done,tap(5).done]],
	[show(testP1C,less), 1, ["TRUE"]],
	[rest, 1, [start]]
	]).

schema(testP1NC, [
	[setOnce(set(start, 0, tool(tap), 1, currentTool, tap, hcStream.ignore, 1)), 1, ["TRUE"]], %INIT
	%[monitorWpNC, 1, ["TRUE"]],
	[report(testP1NC), 1, ["TRUE"]],
	[hcWaypoint(start), 1, [-start]],
	[setOnce(set(start, 1)), 1, [-start, waypoint(start).reached]],
	[testPlan1, 1, [start]],
	[setOnce(set(executeReport,1)), 1, [start]],
	[setOnce(set(executeReport, 0)), 1, [tap(1).done, tap(2).done,tap(3).done,tap(4).done,tap(5).done]],
	[show(testP1NC,less), 1, ["TRUE"]],
	[restNC, 1, [start]]
	]).

schema(testPlan2, [
	%[setOnce(set(tool(tap), 1, currentTool, tap)), 1, ["TRUE"]], %TODO remove
	[tappingTask(1), 1, ["TRUE"]],
	[tappingTask(2), 1, ["TRUE"]],
	[applyBigHeadTask2(1), 1, [tap(1).done]],
	[applyBigHeadTask2(2), 1, [tap(2).done]],
	[rest, 1, ["TRUE"]]
	]).

schema(testP2C, [
	[setOnce(set(start, 0, tool(tap), 1, currentTool, tap)), 1, ["TRUE"]], %INIT
	[monitorWp, 1, ["TRUE"]],
	[hcStream, 1, ["TRUE"]],
	[report(testP2C), 1, ["TRUE"]],
	[hcWaypoint(start), 1, [-start]],
	[setOnce(set(start, 1)), 1, [-start, waypoint(start).reached]],
	[testPlan2, 1, [start]],
	[setOnce(set(executeReport,1)), 1, [start]],
	[setOnce(set(executeReport, 0)), 1, [tap(1).done, tap(2).done, applyBigHead(1).done, applyBigHead(2).done]]
	]).

schema(test2PNC, [
	[setOnce(set(start, 0, tool(tap), 1, currentTool, tap)), 1, ["TRUE"]], %INIT
	[monitorWpNC, 1, ["TRUE"]],
	[report(testP2NC), 1, ["TRUE"]],
	[hcWaypoint(start), 1, [-start]],
	[setOnce(set(start, 1)), 1, [-start, waypoint(start).reached]],
	[testPlan2, 1, [start]],
	[setOnce(set(executeReport,1)), 1, [start]],
	[setOnce(set(executeReport, 0)), 1, [tap(1).done, tap(2).done, applyBigHead(1).done, applyBigHead(2).done]]
]).

schema(test,[
	%[hcWaypoint(start),1,["TRUE"]]
	[hcWaypoint(left), 1, [-start]],
	[setOnce(set(start, 1, executeReport, 1)), 1, [waypoint(left).reached]],
	[report(prova), 1, [start]],
	[hcWaypoint(right),1,[start]],
	[hcWaypoint(up), 1, [waypoint(right).reached]],
	[hcWaypoint(down), 1, [waypoint(up).reached]],
	[setOnce(set(executeReport, 0, start, 0)), 1, [waypoint(down).reached]]
	%[forget(test), 1, [waypoint(right).reached, -executeReport, -start]]
	]).

schema(rest, [
	[hcWaypoint(rest, true), 1, [-busy]]
	]).

schema(restNC, [
	[hcWaypoint(rest.approach), 1, ["TRUE"]]
	]).

schema(testd, [
	[setOnce(set(start, 0, tool(tap), 1, currentTool, tap)), 1, ["TRUE"]], %INIT
	[setOnce(set(start,1)), 1, [waypoint(start).reached]],
	[monitorWp, 1, ["TRUE"]],
	[testPlan1, 1, [start]],
	[show(testd,less), 1, ["TRUE"]],
	[rest, 1, ["TRUE"]]
	]).

schema(testb, [
	[setOnce(set(start, 0, tool(glueDispenser), 1, currentTool, glueDispenser)), 1, ["TRUE"]], %INIT
	[setOnce(set(start,1, tap(1).done, 1, tap(2).done, 1, executeReport, 1)), 1, [waypoint(start).reached]],
	[monitorWp, 1, ["TRUE"]],
	[report(prova_b), 1, [start]],
	[testPlan2, 1, [start]],
	[show(testb,less), 1, ["TRUE"]],
	[rest, 1, ["TRUE"]],
	[setOnce(set(executeReport, 0)), 1, [applyBigHead(1).done, applyBigHead(2).done]]
	]).


% Experiment 2
schema(experiment2,
	[
		[monitorWp, 1, ["TRUE"]],
		[hcStream, 1, ["TRUE"]],
		[tappingTask(1), 1, ["TRUE"]],
		[tappingTask(2), 1, ["TRUE"]],
		[tappingTask(3), 1, ["TRUE"]],
		[tappingTask(4), 1, ["TRUE"]],
		[tappingTask(5), 1, ["TRUE"]]
	]).

schema(experiment3,
	[
		%[monitorWp, 1, ["TRUE"]],
		%[hcStream, 1, ["TRUE"]],
		[tappingTask(1), 1, ["TRUE"]],
		[tappingTask(2), 1, ["TRUE"]],
		%[tappingTask(4), 1, ["TRUE"]],
		%[applyBigHeadTask(1), 1, ["TRUE"]],
		[applyBigHeadTask(1), 1, [tap(1).done]],
		[applyBigHeadTask(2), 1, [tap(2).done]],
		%[applyBigHeadTask(4), 1, [tap(4).done]],
		[rest, 1, ["TRUE"]]
	]).


%HC: humanContact
schema(hcStream,[]).
schema(monitorWp, []).
schema(monitorWpNC, []).
schema(hcWaypoint(_),[]).
schema(hcWaypoint(_,_),[]).
schema(wait(_,_), []). %wait for input and set variable 
schema(wait(_, _, _),[]). %set a starting value, wait for input and then set variable
schema(setOnce(_),[]).
schema(setMultiple(_, _, _, _),[]).
%schema(hcWaypoint(_, _, _, _),[]).

%HC - end
%schemi CONCRETI (ie. behavior)
schema(wander,[]).
schema(forget(_),[]).
schema(sonarStream,[]).
schema(engineStream,[]).
schema(blobStream,[]).
schema(inputStream,[]).
schema(remember(_),[]).
schema(listing,[]).
schema(avoid,[]).
schema(gotoxy(_,_,_),[]).
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
schema($(_),[]).

schema(set(_,_,_),[]).
schema(set(_,_),[]). %means only once
schema(set(_, _, _, _), []). %id to create more instances
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
% GOAL ICOSAF
goal(approach(ID), [waypoint(ID.approach).reached]).
goal(applyBigHead(ID), [applyBigHead(ID).done]).
goal(applyBigHeadTask(ID), [applyBigHead(ID).done]).
goal(applyBigHeadTask2(ID), [applyBigHead(ID).done]).
goal(pickBigHead(ID), [bigHead.picked]). %TODO bigHead(ID).picked
goal(getGlue, [glue.applied]).
goal(applyGlue(ID), [glue(ID).applied]).
goal(getGlue(ID), [glue(ID).applied]).
goal(insertNutzer(ID), [insertNutzer(ID).done]).
goal(insertNutzerTask(ID), [insertNutzer(ID).done]).
goal(tap(ID), [tap(ID).done]).
goal(tappingTask(ID), [tap(ID).done]).
goal(tappingTask2(ID), [tap(ID).done]).
goal(executeTapping(ID), [tap(ID).done]).
goal(removeTool, [tool(none), waypoint(changeToolPos).reached]).
goal(putTool(ID), [tool(ID)]).
goal(changeTool(ID), [tool(ID), -tool(none)]).

%goal(testICOSAF, [applyBigHead(12).done, applyBigHeadTask(32).done]).
%				  applyBigHead(42).done, applyBigHead(52)]).

goal(testICOSAF, [applyBigHead(12).done]).%, tap(2).done, tap(3).done, tap(4).done, tap(5).done]).

goal(testPlan1, [tap(1).done, tap(2).done,tap(3).done,tap(4).done,tap(5).done]).
goal(testPlan2, [tap(1).done, tap(2).done, applyBigHead(1).done, applyBigHead(2).done]).


% GOAL HUMAN CONTACT

goal(hcWaypoint(ID), [waypoint(ID).reached]).
goal(hcWaypoint(ID, true), [waypoint(ID).reached]).


result(S,R):-goal(S,G),effect(S,E),append(G,E,R).
result(S,R):-goal(S,R).
result(S,R):-effect(S,R).

%effect(bwPut(X,Y),[hand.free,-hand.X,-Y.free]).

schemaInstance(S,L):-schema(S,L),instanceList(L).

instanceList([]).
instanceList([[S,_,_]|Rest]):-schemaInstance(S,_),instanceList(Rest).
