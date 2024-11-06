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



%Server

getSemantics(X):-
	yield([],_),
	schema(X,SemDef,_),
	yield(SemDef,_),!.

%added 02/12/2020 in SEED 4.0
getGoal(X):-
	yield([],_),
	schema(X,_,[],_),!,false.

getGoal(X):-
	yield([],_),
	schema(X,_,Goal,_),
	yield(Goal,_),!.
	
%added 08/04/2022 in SEED 5.0
getRegulations(X):-
	yield([],_),
	schema(X,_,_,[]),!,false.
	
getRegulations(X):-
	yield([],_),
	schema(X,_,_,Regulations),
	yield(Regulations,_),!.
	

% Execute schema only ONCE (i.e., forget it on accomplishment)
schema( once(Instance), [[hardSequence([Instance, iiwaWait(iiwaOnce,1), forget(once(Instance))]), 0, ["TRUE"]]], [], [] ).


% Structure of a Schema:
%	a schema has name, list of sub-schemata and goal, all within the schema()
%	predicate. The name can countains parameters that are propagated to the 
%	list and the goal (unified). Each sub-schema countains a default emphasis value
%	and a list of enabling variables (releaer). If the goal is not present, the schema
%	is not teleological.
%		NOTE: the goal within the schema predicate was added 02/12/2020 in SEED 4.0.
%		NOTE: the regulations within the schema predicate was added 08/04/2022 in SEED 5.0.
% eg:
%	schema( name(Param), [[subschema1(Param),emph,[releaserList]], [subschema2...]...], goal, regulations ).

%% ALIVE SCHEMA

%alive: this is the actual SEED loaded by default in the WM. The subnodes of alive are loaded on start 
schema(alive, [
	[inputStream,0,["TRUE"]],
	[rosStream,0,["TRUE"]],
	%[show(alive),0,["TRUE"]],
	[memory,0,["TRUE"]],
	[wsg50Manager,0,["TRUE"]],
	[iiwaManager,0,["TRUE"]],
	[requestStream,0,["TRUE"]] ],
	[],
	[] ).
	
%% ABSTRACT SCHEMATA

%add q to kill SEED
schema( q, [[forget(alive),0, ["TRUE"]]], [], [] ).

% IIWA tasks (abstract)

schema( iiwa\teach\Act, [
	[iiwaGo(teach),1,["TRUE"]],
	[iiwaTeach(Act),0,[teach.near]] ] , 
	[Act.known], 
	[] ).

schema( iiwa\try\Act\M, [
	[iiwaExe(Act,M),0,[Act.known]],
	[iiwa\teach\Act,0,[-Act.known]] ], 
	[iiwaExe(Act,M).done], 
	[] ).

schema(k, [[set(teaching.done,true),0,["TRUE"]]], [], [] ).

% --




%% CONCRETE SCHEMATA

%forget(X): remove the node X from the WM
schema(forget(_), [], [], [] ).

%remember(X,Y): add the node X to the Y node in WM
schema(remember(_,_), [], [], [] ).
%remember(X): add the node X to the "alive" node in WM (default)
schema(remember(_), [], [], [] ).

%inputStream: enable commands from keyboard
schema(inputStream, [], [], [] ).

%listing: plot all nodes of the WM
schema(listing, [], [], [] ).

%show(X): publish on the topic /seed_SEEDNAME/show the image of the WM (graphviz based)
schema(show(_), [], [], [] ).
%show(X,less): publish a compact version of the WM (only enabled nodes are plot)
schema(show(_,less), [], [], [] ).

%gui: open the Graphical User Interface of the SEED (QT based)
schema(gui, [], [], [] ).

%requestStream: standard anchestor for the requested nodes
schema(requestStream, [], [], [] ).

%test: behavior used for node-testing
schema(test, [], [], [] ).

%rosStream: interface with the ROS environment (enables topics and services)
schema(rosStream, [], [], [] ).

%joyStream: interface with the Joypad
schema(joyStream, [], [], [] ).

%ltm(X): post a SwiProlog Query on this file
schema(ltm(_), [], [], [] ).

%set(X,V,P): set the WM variable X to a value V with a specific period P
schema(set(_,_,_), [], [], [] ).
%set(X,V): set the WM variable X to a value V only once
schema(set(_,_), [], [], [] ).

%get(X,T,P): get the WM variable X of type T with a specific period P
schema(get(_,_,_), [], [], [] ).
%get(X,T): get the WM variable X of type T only once
schema(get(_,_), [], [], [] ).

%timer(X,V,W): set the WM variable X to a value V after W seconds (waiting time)
schema(timer(_,_,_), [], [], [] ).

%compete(X,T,V,P): compete to write on a variable X of type T the value V every period P
schema(compete(_,_,_,_), [], [], [] ).
%compete(X,T,V): compete to write on a variable X of type T the value V only once
schema(compete(_,_,_), [], [], [] ).

%solve(X,T,P): solve the competition every P seconds for a variable X of type T (result is plotted in blue)
schema(solve(_,_,_), [], [], [] ).
%solve(X,T): solve the competition only once for a variable X of type T (result is plotted in blue)
schema(solve(_,_), [], [], [] ).

% Added in SEED 6.0
%solve(X,T,R,P): solve the competition every P seconds for a variable X of type T, publish the result on topic R
schema(rosSolve(_,_,_,_), [], [], [] ).
%solve(X,T,R): solve the competition only once for a variable X of type T, publish the result on topic R
schema(rosSolve(_,_,_), [], [], [] ).

%rosAct(ACTION_ID, ACTOR, TOPIC, RATE)
schema(rosAct(_,_,_,_), [], [], [] ).
%rosAct(ACTION_ID, ACTOR, TOPIC)
schema(rosAct(_,_,_), [], [], [] ).

%state(V,R): set to TRUE the value of a variable V and communicate it through the topic R
schema(rosState(_,_), [], [], [] ).

%template(MSG): template behavior used to show how to add behaviors in SEED
schema(template(_), [], [], [] ).


%hardSequence(TASK_LIST): sequential behavior used to implement HARD sequence in SEED
schema(hardSequence(T), [], [hardSequence(T).done], [] ).

%softSequence(TASK_LIST): sequential behavior used to implement SOFT sequence in SEED
schema(softSequence(T), [], [softSequence(T).done], [] ).



% IIWA tasks (concrete)
schema(iiwaManager, [], [], [] ).

schema(iiwaGo(F), [], [iiwaGo(F).done], [] ).

schema(iiwaTeach(Motion), [], [iiwaTeach(Motion).done], [] ).

schema(iiwaExe(Motion,MXY), [], [iiwaExe(Motion,MXY).done], [] ).

schema(iiwaWrite(Word,Frame), [], [iiwaWrite(Word,Frame).done], [] ).

schema(iiwaInsert(Obj1,Frame), [], [inserted(Obj1,Frame)], [] ).

schema(iiwaExtract(Obj1,Frame), [], [extracted(Obj1,Frame)], [] ).



% IIWA tasks (abstract)

schema(iiwaWait(Event), [
	[timer(Event.done,true,3),0,["TRUE"] ] ],
	[Event.done],
	[]).

schema(iiwaWait(Event,T), [
	[timer(Event.done,true,T),0,["TRUE"] ] ],
	[Event.done],
	[]).

schema(iiwaTest1, [
	[hardSequence([%iiwaGo(robot_bottle.1),iiwaWait(w1),
			iiwaGo(robot_bottle.2),iiwaWait(w2),
			iiwaGo(robot_bottle.3),iiwaWait(w3),
			wsg50Grasp(bottle),
			iiwaGo(home)
			%iiwaGo(test1_final)
				]), 0, ["TRUE"] ] ], 
	[], []).
	
schema(iiwaTest2, [
	[hardSequence([%iiwaGo(robot_bottle.1),iiwaWait(w1),
			iiwaGo(robot_bottle_cup.2),iiwaWait(w5),
			iiwaGo(robot_bottle_cup.3),iiwaWait(w6),
			iiwaWait(pouring)]), 0, ["TRUE"] ] ], 
	[], []).


schema(iiwaTest3, [
	[hardSequence([%iiwaGo(robot_bottle_bowl.1),iiwaWait(w4),
			iiwaGo(robot_bottle_bowl.2),iiwaWait(w5),
			iiwaGo(robot_bottle_bowl.3),iiwaWait(w6),
			wsg50Release(bottle)]), 0, ["TRUE"] ] ], 
	[], []).
	
	
	
	
schema(iiwaTest(ball), [
	[hardSequence([%iiwaGo(robot_bottle_bowl.1),iiwaWait(w4),
			iiwaGo(robot_ball.2),iiwaWait(w5),
			iiwaGo(robot_ball.3),iiwaWait(w6),
			wsg50Grasp(ball),
			iiwaGo(home)]), 0, ["TRUE"] ] ], 
	[], []).

schema(iiwaTest(package), [
	[hardSequence([%iiwaGo(robot_bottle_bowl.1),iiwaWait(w4),
			iiwaGo(robot_ball_package.2),iiwaWait(w5),
			iiwaGo(robot_ball_package.3),iiwaWait(w6),
			wsg50Release(ball)]), 0, ["TRUE"] ] ], 
	[], []).
	
	


schema(iiwaTest(pen), [
	[hardSequence([%iiwaGo(robot_bottle_bowl.1),iiwaWait(w4),
			iiwaGo(robot_pen.2),iiwaWait(w5),
			iiwaGo(robot_pen.3),iiwaWait(w6),
			wsg50Grasp(pen),
			iiwaGo(home)]), 0, ["TRUE"] ] ], 
	[], []).

schema(iiwaTest(cup), [
	[hardSequence([%iiwaGo(robot_bottle_bowl.1),iiwaWait(w4),
			iiwaGo(robot_pen_cup.2),iiwaWait(w5),
			iiwaGo(robot_pen_cup.3),iiwaWait(w6),
			wsg50Release(pen)]), 0, ["TRUE"] ] ], 
	[], []).




schema(iiwaUnstack(Obj1,Obj2), [
	[hardSequence([iiwaDetach(Obj1,Obj2),iiwaWait(w0),iiwaPut(Obj1,gnd)]), 0, ["TRUE"] ] ], 
	[], []).


schema(iiwaStack(Obj1,Obj2), [
	[hardSequence([iiwaPick(Obj1),iiwaWait(w0),iiwaAttach(Obj1,Obj2)]), 0, ["TRUE"] ] ], 
	[], []).
	
	

schema(iiwaPick(Obj), [
	[hardSequence([iiwaGo(Obj.pre),iiwaGo(Obj),iiwaWait(w1),wsg50Grasp(Obj),iiwaGo(home),timer(Obj.taken,true,0.1)]), 0, ["TRUE"] ] ], 
	[Obj.taken], []).
%schema(iiwaPick(Obj1,Obj2), [
%	[hardSequence([iiwaGo(Obj1.pre),iiwaGo(Obj1),iiwaWait(w1),wsg50Grasp(Obj1),iiwaGo(Obj2.pre),iiwaGo(home),timer(Obj1.taken,true,0.1)]), 0, ["TRUE"] ] ], 
%	[Obj1.taken], []).

	
schema(iiwaPut(Obj1,Obj2), [
	[hardSequence([iiwaGo(Obj2.pre),iiwaGo(Obj2.over),iiwaWait(w3),wsg50Release(Obj1),iiwaGo(home),timer(on(Obj1,Obj2),true,0.1)]), 0, ["TRUE"] ] ], 
	[on(Obj1,Obj2)], []).



schema(iiwaAttach(Obj1,Obj2), [
	[hardSequence([iiwaGo(Obj2.pre),iiwaWait(w2),iiwaInsert(Obj1,Obj2.over),iiwaWait(w3),wsg50Release(Obj1),iiwaGo(home),timer(on(Obj1,Obj2),true,0.1)]), 0, ["TRUE"] ] ], 
	[on(Obj1,Obj2)], []).
	
schema(iiwaDetach(Obj1,Obj2), [
	[hardSequence([iiwaGo(Obj1.pre),iiwaGo(Obj1),iiwaWait(w2),wsg50Grasp(Obj1),iiwaExtract(Obj1,Obj2.pre),iiwaWait(w3),iiwaGo(home),timer(Obj1.taken,true,0.1)]), 0, ["TRUE"] ] ], 
	[Obj1.taken], []).


% --


% vision
schema(visionStream, [], [], [] ).


% WSG50 tasks (concrete)

schema(wsg50Manager, [], [], [] ).

schema(wsg50Grasp(Obj), [], [gripper.hold(Obj)], [] ).

schema(wsg50Release(Obj), [], [-gripper.hold(Obj)], [] ).

% --


