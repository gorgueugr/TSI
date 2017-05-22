(define (domain BELKAN2)
	 (:requirements :strips :typing :adl :fluents)
	 (:types robot zona objeto personaje orientacion)
	 (:predicates
	 		(atR ?x - robot ?y - zona)
			(atO ?x - objeto ?y - zona)
			(atP ?x - personaje ?y - zona)
			(orientado ?r - robot ?o - orientacion)
			(conectada ?x ?y - zona ?z - orientacion)
			(cogido ?x - objeto)
			(tiene ?x - objeto ?y - personaje)
			(manoVacia)
			(izq ?z1 ?z2 - orientacion)
	 )
	 (:functions
		 (coste-total)
		 (coste ?z1 ?z2 - zona)
		 )
	(:action girar-izq
	:parameters (?r - robot ?o ?o1 - orientacion)
	:precondition(and(orientado ?r ?o)(izq ?o ?o1))
	:effect(and (not (orientado ?r ?o))(orientado ?r ?o1))
	)
	(:action girar-der
	:parameters (?r - robot ?o ?o1 - orientacion)
	:precondition(and (orientado ?r ?o)(izq ?o1 ?o))
	:effect(and (not (orientado ?r ?o))(orientado ?r ?o1))
	)
	(:action mover-a
		:parameters (?r - robot ?z1 ?z2 - zona ?o - orientacion)
		:precondition (and (not (atR ?r ?z2))(atR ?r ?z1)(orientado ?r ?o)(conectada ?z1 ?z2 ?o))
		:effect (and
							(not (atR ?r ?z1))
							(atR ?r ?z2)
							(increase (coste-total)(coste ?z1 ?z2))
						)
	)
	(:action coger-objeto
		:parameters (?r - robot ?obj - objeto ?z - zona)
		:precondition (and (atR ?r ?z) (atO ?obj ?z)(manoVacia))
		:effect (and
							(not (manoVacia))
							(cogido ?obj)
							(not (atO ?obj ?z))
			)

	)
	(:action dejar-objeto
		:parameters (?r - robot ?obj - objeto ?z - zona)
		:precondition (and (atR ?r ?z)(cogido ?obj))
		:effect (and
							(not (cogido ?obj))
							(atO ?obj ?z)
							(manovacia)
							(atO ?obj ?z)
					)
	)
	(:action entregar-objeto
		:parameters(?r - robot ?obj - objeto ?z - zona ?p - personaje)
		:precondition (and (not (tiene ?obj ?p))(atR ?r ?z)(atP ?p ?z)(cogido ?obj))
		:effect (and (not (cogido ?obj))(tiene ?obj ?p)(manovacia))
	)
)
