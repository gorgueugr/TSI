(define (domain monkey-domain)	       ; Comment: adding location caused fail
  (:requirements :strips :equality :typing)
  (:types  monkey banana box - locatable
          location)
 
  (:predicates 
	       (on-floor ?x - monkey )
	       (at ?m - locatable ?x - location)
	       (onbox ?x - monkey ?y - location)
	       (hasbananas ?x - monkey)
	       
	       )
  (:action GRAB-BANANAS
	     :parameters (?m - monkey ?y - location)
	     :precondition (and (onbox ?m  ?y))
	     :effect (hasbananas ?m))
)