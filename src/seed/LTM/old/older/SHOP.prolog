
shop([],_,[]).

shop([Op|TaskList],S,[Op|PlanList]):-
	run(Op,S,Snew),
	shop(TaskList,Snew,PlanList).

shop([Met|TaskList],S,PlanList):-
	method(Met,ConstList,MetList),
	listMember(ConstList,S),
	append(MetList,TaskList,NewTaskList),
	shop(NewTaskList,S,PlanList).



run(Task,S,Snew):-
	operator(Task,Delete,Add),
	stateDelete(Delete,S,Sclear),
	stateAdd(Add,Sclear,Snew).

stateDelete([],State,State).

stateDelete([Elem|DeleteList],State,NewState):-
	delete(Elem,State,StateClear),
	stateDelete(DeleteList,StateClear,NewState).

stateDelete([_|DeleteList],State,NewState):-
	stateDelete(DeleteList,State,NewState).

stateAdd([],State,State).

stateAdd([Elem|AddList],State,NewState):-
	ground(Elem),
	member(Elem,State),
	stateAdd(AddList,State,NewState).

stateAdd([Elem|AddList],State,[Elem|NewState]):-
	ground(Elem),
	stateAdd(AddList,State,NewState).

listMember([],_).

listMember([Elem|ToCheck],List):-
	member(Elem,List),
	listMember(ToCheck,List).


%%  method(TASK,CONSTRAINTlist,SUBTASKlist)

method(take(X),[on(X,P),on(Y,X),free(Z),at(me,P)],[pickup(Y),place(Y,Z),take(X)]).
method(take(X),[on(X,P),on(Y,X),free(Z)],[goto(P),pickup(Y),place(Y,Z),take(X)]).
method(take(X),[on(X,Y),at(me,Y)],[pickup(X)]).
method(take(X),[on(X,Y)],[goto(Y),pickup(X)]).

method(place(X,Y),[inhand(X),free(Y),at(me,Y)],[putdown(X,Y)]).
method(place(X,Y),[inhand(X),free(Y)],[goto(Y),putdown(X,Y)]).


%%  operator(TASK,DELETElist,ADDlist)

operator(goto(X),[at(me,_)],[at(me,X)]).
operator(pickup(X),[emptyhand,on(X,Y)],[inhand(X),free(Y)]).
operator(putdown(X,Y),[inhand(X),free(Y)],[emptyhand,on(X,Y)]).


