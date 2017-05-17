(define (domain logistics)
	(:requirements :adl :typing )
	(:types physobj location city - object
		obj vehicle - physobj
		truck airplane - vehicle
		airport - location)
	(:predicates 	(at ?x - physobj ?l - city)
			(in ?x - obj ?t - vehicle)
			(connected ?city1 ?city2 - city))
			
	(:action drive-truck
	:parameters (?truck - truck ?city1 ?city2 - city)
	:precondition (and  (at ?truck ?city1)
			            (connected ?city1 ?city2))
	:effect (and  (at ?truck ?city2)
			      (not (at ?truck ?city1))
			(forall (?x - obj)
			  (when (and (in ?x ?truck))
			        (and (not (at ?x ?city1))
				         (at ?x ?city2))
		           )
			)
		    )
    )
)