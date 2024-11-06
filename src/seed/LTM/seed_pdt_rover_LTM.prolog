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
	
getSchemaFromEffect(X):-
	yield([],_),
	findall(Schema,includeEffect(Schema,X),SchemaList),
	yield(SchemaList,_),!.

includeEffect(Schema,X):-
	result(Schema,Result),
	subset(X,Result),
	ground(Schema).


% Structure of a Schema:
%	a schema has name, list of sub-schemata and goal, all within the schema()
%	predicate. The name can countains parameters that are propagated to the 
%	list and the goal (unified). Each sub-schema countains a default emphasis value
%	and a list of enabling variables (releaer). If the goal is not present, the schema
%	is not teleological.
%		NOTE: the goal within the schema predicate was added 02/12/2020 in SEED 4.0.
% eg:
%	schema(name(Params), [
%		[subschema1(Param),emph,[releaserList]], 
%		[subschema2...]... ], 
%		goal, 
%		regulations ).

%% ALIVE SCHEMA

%alive: this is the actual SEED loaded by default in the WM. The subnodes of alive are loaded on start 
schema(alive,[
	[inputStream,0,["TRUE"]],
	[rosStream,0,["TRUE"]],
	[ldc_rover_task,0,["TRUE"]],%[start,-stop]],
	%[gui,0,["TRUE"]],
	[show(ldc_rover_task),0,["TRUE"]],
	[memory,0,["TRUE"]],
	[requestStream,0,["TRUE"]] ],
	[],
	[] ).
	
%% ABSTRACT SCHEMATA

%add q to kill SEED
schema(q, [[forget(alive),0,["TRUE"]]], [], [] ).

%%-- Leonardo Comeptition Drone Domain

% HOW TO RUN:
%	TERMINAL1: ros2 run seed seed pdt_rover
%	TERMINAL1: gui
%	TERMINAL1: lnd_terrestrial
%	TERMINAL3: ros2 topic echo /seed_pdt_rover/command
%
% USE TOPICS/SERVICES ON TERMINAL3 TO CHANGE THE STATE OF THE SYSTEM:
%	(eg1) ros2 service call /seed_lnd_rover/set_regulation_value seed_interfaces/SetRegulationValue "{source: 'map.unknown', value: 0.2}"
%	(eg2) ros2 topic pub /seed_lnd_rover/regulation std_msgs/String "data: map.unknown 0.2" --once
%	(eg3) ros2 topic pub /seed_lnd_rover/state std_msgs/String "data: target.found" --once

schema(lnd_terrestrial,[
	[tfobserver,0,["TRUE"]],
	%[patrol\and\return,1,[-error,drone.flying]]],
	[patrol\and\return,1,[-error]]],
	[home.reached, mapping.done, target.followed],
	[] ).

schema(patrol\and\return,[
	[map\and\seek,1,["TRUE"]],
	[goto(home),1,[mapping.done, target.followed]] ],
	[home.reached, mapping.done, target.followed],
	[] ).
	
schema(map\and\seek,[
	[explore,0,["TRUE"]],
	%[follow(target),0,[target.found]] ],
	[team\picture,0,[target.found]] ],
	[mapping.done, target.followed],
	[] ).
	
schema(team\picture,[
	[goto(target),0,["TRUE"]] ],
	[picture.done],
	[] ).
	
	
% new LDC tasks (2024)

schema(ldc_rover_task,[
	[tfobserver,0,["TRUE"]],
	[team\follow\path,0,[team.path.enabled]],
	[team\return\base,0,[team.return.enabled]],
	%[team\picture,0,["TRUE"]],
	[explore,0,["TRUE"]] ],
	[],
	[] ).

schema(team\follow\path,[
	[meet\at\path0,0,["TRUE"]],
	[followPath,0,[drone.path0.ready,rover.path0.ready]], 
	[rosState(path.accomplished,seed/state),0,[followPath.done]] ],
	[path.accomplished],
	[] ).
	
schema(meet\at\P,[
	[goto(P),0,["TRUE"]],
	[wait,0,[P.reached]],
	[rosState(rover.P.ready,seed/state),0,[P.reached]] ],
	[rover.P.ready,drone.P.ready],
	[] ).
	
schema(team\return\base,[
	[goto(home),0,["TRUE"]],
	[wait,0,[home.reached]],
	[rosState(rover.returned.home,seed/state),0,[home.reached]] ],
	[rover.returned.home,drone.returned.home],
	[] ).
	
schema(team\picture,[
	[goto(picture),0,["TRUE"]],
	[wait,0,[picture.reached]],
	[rosState(rover.picture.ready,seed/state),0,[picture.reached]] ],
	[picture.accomplished], %NOTE: this must be communicated by PTZ
	[] ).

% CONCRETE:

schema(takeoff, [], [-landed], [] ).

schema(land, [], [landed], [] ).

schema(explore, [], [explore.done], [] ).

schema(flyto(X), [], [X.reached], [X.distance] ).

schema(goto(target), [], [], [target.distance] ).

schema(goto(X), [], [X.reached], [X.distance] ).

schema(follow(T), [], [T.followed], [T.distance] ).

schema(wait, [], [], [] ).

schema(tfobserver, [], [], [] ).

%%--



%% CONCRETE SCHEMATA

%forget(X): remove the node X from the WM
schema(forget(_), [], [], [] ).

%remember(X,Y): add the node X to the Y node in WM
schema(remember(X,Y), [], [Y.remember.X], [] ).
%remember(X): add the node X to the "alive" node in WM (default)
schema(remember(X), [], [alive.remember.X], [] ).

%inputStream: enable commands from keyboard
schema(inputStream, [], [], [] ).

%listing: plot all nodes of the WM
schema(listing, [], [], [] ).

%NOT USED
schema(emp(_), [], [], [] ).

%show(X): publish on the topic /seed_SEEDNAME/show the image of the WM (graphviz based)
schema(show(_), [], [], [] ).
%show(X,less): publish a compact version of the WM (only enabled nodes are plot)
schema(show(_,less), [], [], [] ).

%gui: open the Graphical User Interface of the SEED (QT based)
schema(gui, [], [], [] ).

%requestStream: standard anchestor for the requested nodes
schema(requestStream, [], [], [] ).

%memory: standard anchestor for the nodes to be remembered (ABSTRACT FOR NOW)
schema(memory, [], [], [] ).

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

%rosState(V,R): set to TRUE the value of a variable V and communicate it through the topic R
schema(rosState(_,_), [], [], [] ).

%rosObs(N,Op,Fun,T1,T2,Var): set Var = N Op Fun(T1,T2), where: 
%	N is a numerical param
%	Op is a operator in {">","<","*","/"}
%	Fun is a function in {xdiff,ydiff,zdiff,eudist,xydist}
%	T1,T2 are names of Tf frames
%	Var is the WMV to be set
schema(rosObs(_,_,_,_,_,_), [], [], [] ).




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
