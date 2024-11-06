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
%	schema( name(Param), [[subschema1(Param),emph,[releaserList]], [subschema2...]...], goal ).

%% ALIVE SCHEMA

%alive: this is the actual SEED loaded by default in the WM. The subnodes of alive are loaded on start 
schema(alive,[
	[inputStream,0,["TRUE"]],
	[rosStream,0,["TRUE"]],
	%[show(alive),0,["TRUE"]],
	[manage\rover2\devices,0,["TRUE"]],
	[memory,0,["TRUE"]],
	[requestStream,0,["TRUE"]] ],
	[],
	[] ).
	
%% ABSTRACT SCHEMATA

%add q to kill SEED
schema(q, [[forget(alive),0,["TRUE"]]], [], [] ).

%%-- LunarAnts Domain
schema(manage\Agent\devices,[
	[solve(rover.command,string,Agent/command,1),0,["TRUE"]],
	[solve(rover.sensor,string,Agent/s1,1),0,["TRUE"]],
	[solve(rover.drill,string,Agent/d1,1),0,["TRUE"]] ],
	[],
	[] ).
	
schema(scan(Sensor,Drill,Point),[
	[compete(rover.command,string,exe_goto\Point\scan,0.1),0,["TRUE"]],
	[wait_drill_and_scan(Sensor,Drill,Point),0,[Point.reached]] ],
	[scan(Sensor,Drill,Point).done],
	[distance(Point)] ).
	
schema(wait_drill_and_scan(Sensor,Drill,Point),[
	[state(Point.Sensor.ready,seed/state),0,["TRUE"]],
	%[compete(rover.command,string,wait,0.1),0,[-Point.Drill.ready]],
	[compete(rover.command,string,wait,0.1),0,["TRUE"]],
	[compete(rover.sensor,string,exe_scan\Sensor\Point,0.1),0,[Point.Drill.ready]],
	[remember( state(scan(Sensor,Drill,Point).done, seed/state), memory ),0,[scan(Sensor,Point).done]] ],
	[scan(Sensor,Drill,Point).done],
	[] ).
	
	

schema(drill(Sensor,Drill,Point),[
	[compete(rover.command,string,exe_goto\Point\drill,0.1),0,["TRUE"]],
	[wait_scan_and_drill(Sensor,Drill,Point),0,[Point.reached]] ],
	[scan(Sensor,Drill,Point).done],
	[distance(Point)] ).
	
schema(wait_scan_and_drill(Sensor,Drill,Point),[
	%[compete(rover.command,string,wait,0.1),0,[-Point.Sensor.ready]],
	[compete(rover.command,string,wait,0.1),0,["TRUE"]],
	[state(Point.Drill.ready,seed/state),0,[Point.Sensor.ready]],
	[compete(rover.drill,string,exe_drill\Drill\Point,0.1),0,[Point.Sensor.ready]] ],
	[scan(Sensor,Drill,Point).done],
	[] ).


% CONCRETE:

%schema(compete(rover.command,string,goto_pose(Point),_),[],[Point.reached]). %THIS IS NOT WORKING, THE "." OPERATOR MUST BE AVOIDED
schema(compete(_,string,exe_goto\Point\_,_), [], [Point.reached], [] ).

schema(compete(_,string,exe_scan\Sensor\Point,_), [], [scan(Sensor,Point).done], [] ).
%TEST:  compete(rover.sensor,string,exe_scan s1 p1,0.1)

schema(compete(_,string,exe_drill\Drill\Point,_), [], [drill(Drill,Point).done], [] ).
%TEST:  compete(rover.drill,string,exe_drill d1 p1,0.1)

%schema(requestStream, [[drill(s1,d1,p1),0,["TRUE"]], [drill(s1,d1,p3),0,["TRUE"]], [gui,0,["TRUE"]] ], [], [] ).
schema(requestStream, [[drill(s1,d1,p1),0,[p1.inspected]],[drill(s1,d1,p2),0,[p2.inspected]],[drill(s1,d1,p3),0,[p3.inspected]], [gui,0,["TRUE"]] ], [], [] ).
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

%compete(X,T,V,P): compete to write on a variable X of type T the value V every period P
schema(compete(_,_,_,_), [], [], [] ).
%compete(X,T,V): compete to write on a variable X of type T the value V only once
schema(compete(_,_,_), [], [], [] ).

%solve(X,T,R,P): solve the competition every P seconds for a variable X of type T and publish the value on the ROStopic R 
schema(solve(_,_,_,_), [], [], [] ).

%state(X,T): assume publish the variable X on the topic T  
schema(state(_,_), [], [], [] ).

%schema(freeze(_), [], [], [] ).




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
