%%% RIVISTO 15/12/2016 per GRINGO 4

%% dimensione del cubo
#const e = 2.
%% numero di cubies
#const c = e**3.
%% upperbound mosse
#const t = 8.

%% posizione dei cubetti
cubie(0..c-1).
%% def colori
color(red;blue;green;yellow;orange;white).

time(0..t).

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

%% Solo una mossa per instante t, fino a quando non è risolto il cubo.
1 { move(T1,R,S,D) : rotation(R), side(S), dir(D) } 1 :- 
    time(T1), time(T2), not solved(T2), T2 <= T1.
:- time(T1), time(T2), solved(T2), T2 < T1, move(T1,R,S,D).

%% Vieta 3 rotazioni uguali nello stesso senso.
:- move(T,R,S,D), move(T+1,R,S,D), move(T+2,R,S,D),
   rotation(R), side(S), dir(D), time(T), time(T+1), time(T+2).

%% Vieta la rotazione di 180 gradi con mosse negative.
:- move(T,R,S,-1), move(T+1,R,S,-1),
   rotation(R), side(S),  time(T), time(T+1).

%%Vieta la rotazione di due facce adiacenti nella stessa direzione
:- move(T,R,S1,D), move(T+1,R,S2,D),
   rotation(R), side(S1;S2), dir(D), time(T), time(T+1), S1 != S2.

%%Vieta due rotazione in un senso e poi nel senso inverso dell'altro
%:- move(T,R,S1,D1), move(T+1,R,S1,D1), move(T+2,R,S2,D2), move(T+3,R,S2,D2),
 %  rotation(R), side(S1;S2), dir(D1), dir(D2), time(T), time(T+1), time(T+2), time(T+3), D1 != D2, S1 != S2.

%:- move(T,R,S1,D1), move(T+1,R,S2,D2), move(T+2,R,S2,D2), move(T+3,R,S1,D1),
  % rotation(R), side(S1;S2), dir(D1), dir(D2), time(T), time(T+1), time(T+2), time(T+3), D1 != D2, S1 != S2.

%:- move(T,R,S1,D1), move(T+1,R,S2,D2), move(T+2,R,S1,D1), move(T+3,R,S2,D2),
  % rotation(R), side(S1;S2), dir(D1), dir(D2), time(T), time(T+1), time(T+2), time(T+3), D1 != D2, S1 != S2.


%% Elimina sequenze di mosse che formano dei cicli nella sequenza di stati
%% (sembra piuttosto pesante come controllo)
:- equal(T1,T2), time(T1;T2), T1 < T2.
equal(T1,T2) :- not different(T1,T2), time(T1), time(T2), T1 < T2.
different(T1,T2) :- cubie(X), is(T1,X,Col1,_,_),is(T2,X,Col2,_,_), time(T1), time(T2), color(Col1;Col2), Col1 != Col2, T1 < T2.
different(T1,T2) :- cubie(X), is(T1,X,_,Col1,_),is(T2,X,_,Col2,_), time(T1), time(T2), color(Col1;Col2), Col1 != Col2, T1 < T2.
different(T1,T2) :- cubie(X), is(T1,X,_,_,Col1),is(T2,X,_,_,Col2), time(T1), time(T2), color(Col1;Col2), Col1 != Col2, T1 < T2.

%% In alternativa si può usare un più semplice controllo che vieta una seguenza di due mosse, l'una opposta all'altra.
 :- move(T,R,S,D), move(T1,R,S,D1),
    T1 = T + 1, time(T;T1),
    dir(D;D1), D = -D1,
    rotation(R),side(S).

%% Evoluzione dello stato.
is(T+1,C1,Col3,Col2,Col1) :- is(T,C,Col1,Col2,Col3),%
                            move(T,yaw,S,1),reach(yaw,S,C,C1),%
                            time(T),time(T+1), 
                            cubie(C),cubie(C1),
                            color(Col1), color(Col2), color(Col3),
                            side(S).

is(T+1,C1,Col3,Col2,Col1) :- is(T,C,Col1,Col2,Col3),%
                            move(T,yaw,S,-1),reach(yaw,S,C1,C),%
                            time(T),time(T+1),%
                            cubie(C),cubie(C1),
                            color(Col1), color(Col2), color(Col3),
                            side(S).

is(T+1,C1,Col1,Col3,Col2) :- is(T,C,Col1,Col2,Col3),%
                            move(T,pitch,S,1),reach(pitch,S,C,C1),%
                            time(T),time(T+1),%
                            cubie(C),cubie(C1), 
                            color(Col1), color(Col2), color(Col3),
                            side(S).

is(T+1,C1,Col1,Col3,Col2) :- is(T,C,Col1,Col2,Col3),%
                            move(T,pitch,S,-1),reach(pitch,S,C1,C),%
                            time(T),time(T+1),%
                            cubie(C),cubie(C1),
                            color(Col1), color(Col2), color(Col3),
                            side(S).

is(T+1,C1,Col2,Col1,Col3) :- is(T,C,Col1,Col2,Col3),%
                            move(T,roll,S,1),reach(roll,S,C,C1),%
                            time(T),time(T+1),%
                            cubie(C),cubie(C1),color(Col1;Col2;Col3),side(S).

is(T+1,C1,Col2,Col1,Col3) :- is(T,C,Col1,Col2,Col3),%
                            move(T,roll,S,-1),reach(roll,S,C1,C),%
                            time(T),time(T+1),%
                            cubie(C),cubie(C1),
                            color(Col1), color(Col2), color(Col3),
                            side(S).

%% catch all
is(T+1,C,Col1,Col2,Col3) :- is(T,C,Col1,Col2,Col3),%
                           move(T,R,S,_), not reach(R,S,C,_), not reach(R,S,_,C),%
                           time(T),time(T+1),%
                           cubie(C), color(Col1), color(Col2), color(Col3),
                           rotation(R),side(S).

%% configurazione risolta del cubo
solved(N) :- is(N,0,Col4,Col1,Col3), is(N,1,Col4,Col1,Col5), is(N,2,Col6,Col1,Col5), is(N,3,Col6,Col1,Col3),
             is(N,4,Col4,Col2,Col3), is(N,5,Col4,Col2,Col5), is(N,6,Col6,Col2,Col5), is(N,7,Col6,Col2,Col3),
             time(N),faces(Col1,Col2,Col3,Col4,Col5,Col6).
solved(N+1) :- time(N),time(N+1),solved(N).

%% limita il check alle sole configurazioni risolte effettivamente ottenibili.
%% Ad esempio se green e blue sono colori opposti, non esisterà mai una configurazione
%% risolta che li vede vicini (ma senza faces/6 la regola sopra controllerebbe anche quello!)
faces(green,blue,red,white,orange,yellow).
faces(Col1,Col2,Col4,Col5,Col6,Col3) :- faces(Col1,Col2,Col3,Col4,Col5,Col6).
faces(Col2,Col1,Col6,Col5,Col4,Col3) :- faces(Col1,Col2,Col3,Col4,Col5,Col6).
faces(Col3,Col5,Col4,Col1,Col6,Col2) :- faces(Col1,Col2,Col3,Col4,Col5,Col6).

%% chiedo che il cubo sia risolto 
solved :- solved(_).
:- not solved.

%% Predicato per distinguere la configurazione iniziale.
init(C,Col1,Col2,Col3) :- is(0,C,Col1,Col2,Col3),cubie(C),color(Col1;Col2;Col3).

%% Minimizzazione ripetto al numero di mosse.
#minimize { T@1,solved : solved(T), time(T)}.

%% INPUT 1 %%
 %is(0,0,red,white,green).
 %is(0,1,white,green,orange).
 %is(0,2,orange,white,blue).
 %is(0,3,green,red,yellow).
 %is(0,4,blue,orange,yellow).
 %is(0,5,white,red,blue).
 %is(0,6,blue,yellow,red).
 %is(0,7,yellow,green,orange).

%% INPUT 2 %%
% is(0,0,yellow,green,orange).
% is(0,1,blue,white,orange).
% is(0,2,yellow,orange,blue).
% is(0,3,blue,yellow,red).
% is(0,4,green,orange,white).
% is(0,5,red,blue,white).
% is(0,6,green,red,yellow).
% is(0,7,red,white,green).

%% INPUT 3 %%
%is(0,0,orange,yellow,green).
%is(0,1,white,green,orange).
%is(0,2,orange,white,blue).
%is(0,3,red,blue,yellow).
%is(0,4,white,blue,red).
%is(0,5,blue,yellow,orange).
%is(0,6,green,white,red).
%is(0,7,yellow,red,green).

%is(0,0,green,orange,yellow).
%is(0,1,red,green,white).
%is(0,2,orange,white,blue).
%is(0,3,orange,white,green).
%is(0,4,white,blue,red).
%is(0,5,red,yellow,blue).
%is(0,6,orange,yellow,blue).
%is(0,7,yellow,red,green).



#show move/4.
#show solved/1.
