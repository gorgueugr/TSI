(define (problem monkey-test1)
    (:domain monkey-domain)
  (:objects p1 p2 p3 p4 - location
            monkey1 - monkey
			box1 - box
			bananas1 - banana)
  (:init 
	
	(onbox monkey1 p2)
	 )
  (:goal (AND (hasbananas monkey1))))