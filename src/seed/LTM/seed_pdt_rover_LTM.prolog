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
	%[gui,0,["TRUE"]],
	[lnd_ground,0,["TRUE"]],
	[memory,0,["TRUE"]],
	[requestStream,0,["TRUE"]] ],
	%[show(requestStream),0,["TRUE"]] ],
	[],
	[] ).
	
%% ABSTRACT SCHEMATA

%add q to kill SEED
schema(q, [[forget(alive),0,["TRUE"]]], [], [] ).

%%-- Leonardo Comeptition Drone Domain

% HOW TO RUN:
%	TERMINAL1: ros2 run seed seed pdt_drone
%	TERMINAL2: ros2 topic pub /seed_pdt_drone/state std_msgs/String "data: landed" --once
%	TERMINAL1: gui
%	TERMINAL1: lnd_aerial
%	TERMINAL3: ros2 topic echo /seed_pdt_drone/command
%
% USE TOPICS/SERVICES ON TERMINAL3 TO CHANGE THE STATE OF THE SYSTEM:
%	(eg1) ros2 service call /seed_lnd_drone/set_regulation_value seed_interfaces/SetRegulationValue "{source: 'map.unknown', value: 0.2}"
%	(eg2) ros2 topic pub /seed_lnd_drone/state std_msgs/String "data: target.found" --once

schema(lnd_test(T,Deadline,ID),[
	[deadline(lnd_test(T,Deadline,ID),Deadline),1,["TRUE"]], 
	[photo(T,once,ID),5,["TRUE"]] ],
	[T.once.accepted],
	[] ).


% deadline task, kill a task when deadline is reached
%	NOTE: this must be added as subtask (it will be forgotten too)
schema(deadline(Task,Deadline),[
	[timer(Task.expired,true,Deadline),1,["TRUE"]],
	[forget(Task),1,[Task.expired]] ],
	[],
	[] ).


% default behaviors, always on during execution independently from the mission
schema(lnd_ground,[
	[tfobserver,1,["TRUE"]] ],
	%[explore,1,[drone.flying]],
	%[patrol\and\return,1,[-error,drone.flying]]],
	%[patrol\and\return,1,[-error]]],
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



%% LEONARDO TASK:


% find object (drone version), priority: 10
%	NOTE: this explores and search for the target
schema(find_object(T,Deadline,ID),[
	[deadline(find_object(T,Deadline,ID),Deadline),1,["TRUE"]],
	[obs(T),1,["TRUE"]],
	[explore,1,[-T.exists]],
	%[goto(T.target),5,[T.exists]], 
	[photo(T,once,ID),10,[T.exists]] ],
	[T.once.confirmed],
	[] ).

% find target (drone version), priority: 1-2
%	NOTE: T.observed must be stated by the GCS after the operator's ok
schema(find_target(T,Z1,Z2,Z3,Z4,Deadline,ID),[
	[deadline(find_target(T,Z1,Z2,Z3,Z4,Deadline,ID),Deadline),1,["TRUE"]],
	[obs(T),1,["TRUE"]],
	[obs(T.target),1,["TRUE"]],
	[cover(Z1,Z2,Z3,Z4),1,[-T.exists]],
	[goto(T.target,observe),2,[T.exists]],
	[photo(T,first,ID),2,[T.target.reached]],
	[timer(T.second.ready,true,2.1),1,[T.first.confirmed]],
	[photo(T,second,ID),2,[T.second.ready]],
	[timer(T.observed,true,0.1),1,[T.second.confirmed]] ],
	[T.observed],
	[] ).

% follow sequence
%	NOTE: this can be implemented through a hardSequence as follows:
%		   hardSequence([fly_by(T1),fly_by(T2), ..., fly_by(TN)])
schema(follow_sequence(S,Deadline,ID),[
	[deadline(follow_sequence(S,Deadline,ID),Deadline),1,["TRUE"]],
	[hardSequence(S,fs),5,["TRUE"]] ],
	[hardSequence(fs).done],
	[] ).

schema(move_by(T,ID),[
	[obs(T),1,["TRUE"]],
	[obs(T.target),1,["TRUE"]],
	[goto(T.target,observe),2,[T.exists]],
	[photo(T,once,ID),2,[T.target.reached]],
	[timer(T.observed,true,0.1),1,[T.once.confirmed]] ],
	[T.observed],
	[] ).

% emergency rtb, priority 20
schema(emergency_rtb(Deadline,ID),[
	[deadline(emergency_rtb(Deadline,ID),Deadline),1,["TRUE"]],
	[goto(rover/map),20,["TRUE"]] ],
	[rover/map.reached],
	[] ).



% CONCRETE:

schema(takeoff, [], [-landed], [] ).

schema(land, [], [landed], [] ).

schema(explore, [], [explore.done], [] ).

schema(photo(X,Type,_), [], [X.Type.confirmed], [X.distance] ).

schema(flyto(circle(X)), [], [circle(X).done], [X.distance] ).

schema(flyto(X, observe), [], [X.observed], [X.distance] ).

schema(flyto(X), [], [X.reached], [X.distance] ).

schema(goto(X, observe), [], [X.observed], [X.distance] ).

schema(goto(X), [], [X.reached], [X.distance] ).

schema(cover(Z1,Z2,Z3,Z4), [], [cover(Z1,Z2,Z3,Z4).done], [] ).

schema(follow(T), [], [T.followed], [T.distance] ).

schema(wait, [], [], [] ).

schema(tfobserver, [], [], [] ).

schema(obs(_), [], [], [] ).

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

%hardSequence(TASK_LIST): sequential behavior used to implement HARD sequence in SEED
schema(hardSequence(_,ID), [], [hardSequence(ID).done], [] ).

%softSequence(TASK_LIST): sequential behavior used to implement SOFT sequence in SEED
schema(softSequence(_,ID), [], [softSequence(ID).done], [] ).

%timer(X,V,W): set the WM variable X to a value V after W seconds (waiting time)
schema(timer(V,true,_), [], [V], [] ).
schema(timer(V,false,_), [], [-V], [] ).
schema(timer(_,_,_), [], [], [] ).




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
