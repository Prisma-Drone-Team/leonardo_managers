
% Strips-like planner.
% Top-level interface.
strips(InitState, GoalList, Plan) :-
	%nl,write('PLANNER START, press enter\n'),get_single_char(_),
	strips2(InitState, GoalList, Plan, [InitState],'>').
	%strips1(InitState, GoalList, [], [], _, RevPlan),
	%reverse(RevPlan, Plan).

%%STRIPS 1, quite bad!

% Base case: All goals satisfied in State.
strips1(State, GoalList, Plan, [], State, Plan) :-
	subset(GoalList, State), %.
	write('\ntrivial state\n'),
	nl,write('press enter'),get_single_char(_).

strips1(State, GoalList, Plan, [Ac|_], State, Plan) :-
	subset(GoalList, State), %.
	write(' Action:  '),write(Ac),write(' executable\n'),
	nl,write('press enter'),get_single_char(_).

% Plan
strips1(State, GoalList, Plan, BadActions, NewState, NewPlan) :-
	member(Goal, GoalList),
	not(member(Goal, State)),    % Find unsatisfied goal.
	write('\nAttempting goal:  '),write(Goal),nl,
	is_add(Ac, Goal),              % Find action that achieves it.
	write(' Choosing Action:  '),write(Ac),
	not(member(Ac, BadActions)), % Do not repeat actions (dangerous).
	write(' -- not a bad action.'),nl,
	get_preclist(Ac, PrecList),      % Find preconds and achieve them.       
	strips1(State, PrecList, Plan, [Ac|BadActions], TmpState1, TmpPlan1),
	apply_rule(Ac, TmpState1, TmpState2),  % Recurse w/new state and goals.
	strips1(TmpState2,GoalList,[Ac|TmpPlan1],BadActions,NewState,NewPlan).

%%STRIPS 2, ric:

% Base case: All goals satisfied in State.
strips2(State, GoalList, [], _, _) :-
	subset(GoalList, State), %.
	write('\ngoal reached\n'). %,
	%nl,write('press enter'),get_single_char(_).

% Plan
strips2(State, GoalList, [Ac|Plan], VisitedStates, DEPSTR) :-
	strips_rule(Ac,_,_,_),
	is_applicable(Ac,State),
	apply_rule(Ac, State, NextState),  % Recurse w/new state and goals.
	not(is_visited(NextState, VisitedStates)),
	term_string(Ac,ASTR),
	string_concat(DEPSTR,ASTR,NewDEPSTR),
	%write(NewDEPSTR),nl,
	strips2(NextState,GoalList,Plan,[NextState|VisitedStates],NewDEPSTR).

%strips2(State, _, _, _) :-
%	write('\nNo more actions for state:\n'),write(State),get_single_char(_),nl,!,false.

%is_applicable
is_applicable(Ac,State):-
	get_preclist(Ac,PrecList),
	subset(PrecList,State).

is_visited(S,VSlist):-
	member(VS,VSlist),
	subtract(S, VS, []),
	subtract(VS, S, []).
	
	


% Apply Rule
apply_rule(Ac, State, NewState) :-
	%write('\nSimulating '),write(Ac),nl,
	%write('From: '), write(State), write('\n----> '),
	get_dellist(Ac, DelList),                     % find deleted props
	subtract(State, DelList, TmpState),       % remove deleted props
	get_addlist(Ac, AddList),                     % find added props
	union(AddList, TmpState, NewState).       % add props to old state
	%write(NewState),
	%nl,write('press enter\n'),get_single_char(_).
   
% Utilities
get_preclist(Action, Plist) :- strips_rule(Action, Plist, _, _).
is_prec(Action, Cond) :- get_preclist(Action, Plist), member(Cond, Plist).

get_addlist(Action, Alist) :- strips_rule(Action, _, Alist, _).
is_add(Action, Cond) :- get_addlist(Action, Alist), member(Cond, Alist).

get_dellist(Action, Dlist) :- strips_rule(Action, _, _, Dlist).
is_del(Action, Cond) :- get_dellist(Action, Dlist), member(Cond, Dlist).

% Pretty-print Init, Goal, and Plan.
plan(InitState, Goal) :-
	time(strips(InitState,Goal,Plan)), %get execution-time also
	write('\n\nInit: '), write(InitState),nl,
	write('Goal: '),write(Goal),nl,
	write('Plan len: '),length(Plan,PL),write(PL),nl,
	write('Plan:\n'),
        writeplan(Plan),nl.

% Pretty-print the plan.
writeplan([]).
writeplan([A|B]):-
  write('       '),write(A),nl,
  writeplan(B).



%%%% DOMAIN %%%%
%% strips_rule(RULE,PRECONDITIONS,ADDLIST,DELETELIST).

%taking X from Y at position P
strips_rule(pick(P,X,Y), 
	[at(robot,P),free(X),free(robot),at(X,P),on(X,Y)], 
	[have(robot,X),free(Y)], 
	[at(X,P),on(X,Y),free(robot)]):-
		pose(P),graspable(X),support(Y),dif(X,Y).


%putting X on Y at P
strips_rule(place(P,X,Y), 
	[at(robot,P),have(robot,X),at(Y,P),free(Y)], 
	[at(X,P),on(X,Y),free(robot)], 
	[have(robot,X),free(Y)]):-
		pose(P),graspable(X),support(Y),dif(X,Y). 


%going from P1 to P2
strips_rule(go(P1,P2), 
	[at(robot,P1)], 
	[at(robot,P2)], 
	[at(robot,P1)]):-
		pose(P1),pose(P2),dif(P1,P2).

%% example of planning problem %%
pose(p1).
pose(p2).
pose(p3).
graspable(a).
graspable(b).
graspable(c).
support(b1).
support(b2).
support(b3).
support(a).
support(b).
support(c).


%NOTE time(X). return the execution time of the predicate X

%sussman
%plan([at(b1,p1),at(b2,p2),at(b3,p3),free(robot),at(robot,p1),at(b,p1),on(b,b1),free(b),free(b2),at(c,p3),at(a,p3),on(c,a),on(a,b3),free(c)],[on(a,b),on(b,c))]).

%simple
%plan([at(b1,p1),at(b2,p2),at(b3,p3),free(robot),at(robot,p1),at(c,p1),on(c,b1),free(c),at(b,p2),on(b,b2),free(b),at(a,p3),on(a,b3),free(a)],[on(a,b),on(b,c),on(c,b3)]).
