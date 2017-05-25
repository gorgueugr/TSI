﻿(define (domain BELKAN6)
	 (:requirements :strips :typing :adl :fluents)
	 (:types entregable zapatillas bikini - objeto
		 			robot zona  personaje orientacion
		  		tipoterreno)
	 (:predicates
	 		(atR ?x - robot ?y - zona)
			(atO ?x - objeto ?y - zona)
			(atP ?x - personaje ?y - zona)
			(orientado ?r - robot ?o - orientacion)
			(conectada ?x ?y - zona ?z - orientacion)
			(cogido ?x - objeto)
			(enMochila ?x - objeto)
			(tiene ?x - objeto ?y - personaje)
			(manoVacia)
			(izq ?z1 ?z2 - orientacion)
			(terreno ?z - zona ?t - tipoterreno)
			(caminable ?t - tipoterreno)
			(habilita ?o - objeto ?t - tipoterreno)
	 )
	 (:functions
		 (objetosEntregables)
		 (puntosRobot)
		 (puntos ?p - personaje ?o - entregable)

		 (tamanioMochila)
		 (cosasMochila)

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
		:parameters (?r - robot ?z1 ?z2 - zona ?o - orientacion ?t - tipoterreno)
		:precondition (and
										(not (atR ?r ?z2))
										(atR ?r ?z1)
										(orientado ?r ?o)
										(conectada ?z1 ?z2 ?o)
										(terreno ?z2 ?t)
										(caminable ?t)
									)
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
							(not (atO ?obj ?z))
							(not (manoVacia))
							(cogido ?obj)
							(forall (?t - tipoterreno) (when (habilita ?obj ?t) (caminable ?t)))
						)
	)
	(:action dejar-objeto
		:parameters (?r - robot ?obj - objeto ?z - zona)
		:precondition (and (atR ?r ?z)(cogido ?obj))
		:effect (and
							(atO ?obj ?z)
							(not (cogido ?obj))
							(manoVacia)
							(forall (?t - tipoterreno)
								(when (habilita ?obj ?t) (not (caminable ?t))))
							)
	)
	(:action entregar-objeto
		:parameters(?r - robot ?obj - entregable ?z - zona ?p - personaje)
		:precondition (and (not (tiene ?obj ?p))(atR ?r ?z)(atP ?p ?z)(cogido ?obj))
		:effect (and
							(not (cogido ?obj))
							(tiene ?obj ?p)
							(manoVacia)
							(increase (puntosRobot) (puntos ?p ?obj))
							(decrease (objetosEntregables) 1)
						)
	)
	(:action guardar-en-mochila
	  :parameters (?x - objeto)
	  :precondition (and (not (manoVacia))(cogido ?x)(>= (- (tamanioMochila) (cosasMochila)) 1))
	  :effect (and
								(enMochila ?x)
								(increase (cosasMochila) 1)
								(not (cogido ?x))
								(manoVacia)
						)
		)
		(:action sacar-de-mochila
		  :parameters (?x - objeto)
		  :precondition (and (manoVacia)(enMochila ?x))
		  :effect (and
									(not (enMochila ?x))
									(decrease (cosasMochila) 1)
									(cogido ?x)
									(not (manoVacia))
							)
			)
)
