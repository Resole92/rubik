#include <incmode>.
#const istop  = "SAT".
#const imin = 1.
#const imax = 14. 
%%% RIVISTO 15/12/2016 per GRINGO 4

%% dimensione del cubo
#const e = 2.
%% numero di cubies
#const c = e**3.

%% posizione dei cubetti
cubie(0..c-1).
%% def colori
color(red;blue;green;yellow;orange;white).

%% le rotazioni sono codificate secondo la terminologia degli angoli di eulero
rotation(yaw;pitch;roll).
side(0;1).
dir(1;-1).

%% Determina la posizione di arrivo di un dato cubetto dopo una rotazione positiva.

reach(yaw,0,0,1). % yaw up
reach(yaw,0,1,2). % yaw up
reach(yaw,0,2,3). % yaw up
reach(yaw,0,3,0). % yaw up

reach(yaw,1,4,5). % yaw down
reach(yaw,1,5,6). % yaw down
reach(yaw,1,6,7). % yaw down
reach(yaw,1,7,4). % yaw down

reach(pitch,0,0,4). % pitch right
reach(pitch,0,4,5). % pitch right
reach(pitch,0,5,1). % pitch right
reach(pitch,0,1,0). % pitch right

reach(pitch,1,3,7). % pitch left
reach(pitch,1,7,6). % pitch left
reach(pitch,1,6,2). % pitch left
reach(pitch,1,2,3). % pitch left

reach(roll,0,0,3). % roll front
reach(roll,0,3,7). % roll front
reach(roll,0,7,4). % roll front
reach(roll,0,4,0). % roll front

reach(roll,1,1,2). % roll back
reach(roll,1,2,6). % roll back
reach(roll,1,6,5). % roll back
reach(roll,1,5,1). % roll back

#program base.
%% Predicato per distinguere la configurazione iniziale.
init(C,Col1,Col2,Col3) :- is(0,C,Col1,Col2,Col3),cubie(C),color(Col1;Col2;Col3).

#program check(t). 
%% configurazione risolta del cubo
solved(t) :- is(t,0,Col4,Col1,Col3), is(t,1,Col4,Col1,Col5), is(t,2,Col6,Col1,Col5), is(t,3,Col6,Col1,Col3),
             is(t,4,Col4,Col2,Col3), is(t,5,Col4,Col2,Col5), is(t,6,Col6,Col2,Col5), is(t,7,Col6,Col2,Col3).

%% chiedo che il cubo sia risolto 
:- not solved(t), query(t).

#program step(t).

%% Solo una mossa per instante t, fino a quando non è risolto il cubo.
1 { move(t,R,S,D) : rotation(R), side(S), dir(D) } 1.

%% Vieta 3 rotazioni uguali nello stesso senso.
:- move(t-2,R,S,D), move(t-1,R,S,D), move(t,R,S,D),
   rotation(R), side(S), dir(D).

%% Vieta la rotazione di 180 gradi con mosse negative.
:- move(t-1,R,S,-1), move(t,R,S,-1),
   rotation(R), side(S).

%Vieta due stati che corrispondono alla stessa configurazione
is_equals_cubie(t,C) :- is(T1,C,Col1,Col2,Col3), is(T2,C,Col1,Col2,Col3), cubie(C), color(Col1;Col2;Col3), T1 > T2.
is_all_equal :- is_equals_cubie(t,0), is_equals_cubie(t,1), is_equals_cubie(t,2), 
                is_equals_cubie(t,3), is_equals_cubie(t,4), is_equals_cubie(t,5), 
                is_equals_cubie(t,6), is_equals_cubie(t,7).
:- is_all_equal.

%% Evoluzione dello stato.
is(t,C1,Col3,Col2,Col1) :-  is(t-1,C,Col1,Col2,Col3),
                            move(t,yaw,S,1),reach(yaw,S,C,C1),
                            cubie(C),cubie(C1),
                            color(Col1), color(Col2), color(Col3),
                            side(S).

is(t,C1,Col3,Col2,Col1) :-  is(t-1,C,Col1,Col2,Col3),
                            move(t,yaw,S,-1),reach(yaw,S,C1,C),
                            cubie(C),cubie(C1),
                            color(Col1), color(Col2), color(Col3),
                            side(S).

is(t,C1,Col1,Col3,Col2) :-  is(t-1,C,Col1,Col2,Col3),
                            move(t,pitch,S,1),reach(pitch,S,C,C1),
                            cubie(C),cubie(C1), 
                            color(Col1), color(Col2), color(Col3),
                            side(S).

is(t,C1,Col1,Col3,Col2) :-  is(t-1,C,Col1,Col2,Col3),
                            move(t,pitch,S,-1),reach(pitch,S,C1,C),
                            cubie(C),cubie(C1),
                            color(Col1), color(Col2), color(Col3),
                            side(S).

is(t,C1,Col2,Col1,Col3) :-  is(t-1,C,Col1,Col2,Col3),
                            move(t,roll,S,1),reach(roll,S,C,C1),
                            cubie(C),cubie(C1),color(Col1;Col2;Col3),
                            color(Col1), color(Col2), color(Col3),
                            side(S).

is(t,C1,Col2,Col1,Col3) :-  is(t-1,C,Col1,Col2,Col3),
                            move(t,roll,S,-1),reach(roll,S,C1,C),
                            cubie(C),cubie(C1),
                            color(Col1), color(Col2), color(Col3),
                            side(S).
%% catch all
is(t,C,Col1,Col2,Col3) :-  is(t-1,C,Col1,Col2,Col3),
                           move(t,R,S,_), not reach(R,S,C,_), not reach(R,S,_,C),
                           cubie(C), 
                           color(Col1), color(Col2), color(Col3),%
                           rotation(R),side(S).

#show move/4.


