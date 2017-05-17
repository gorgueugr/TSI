(define (problem logistics-test1)
        (:domain logistics)
;; make sure these are constants or objects:
;; citya cityb cityc cityd city
(:objects citya cityb cityc cityd - city
          truck1 - truck
          pack1 pack2 pack3 - obj 
          
          )
(:init

(connected citya cityb)
(connected cityb citya)
(connected cityb cityc)
(connected cityc cityb)
(connected citya cityd)
(connected cityd citya)
(connected cityd cityc)
(connected cityc cityd)

(at truck1 citya)
(at pack1 citya)
(at pack2 citya)
(at pack3 citya)
(in  pack1 truck1)
(in  pack2 truck1)
(in  pack3 truck1)

(= (fuel truck1) 100)
(= (consumo-viaje truck1) 20)
(= (total-fuel truck1) 0)
)

(:goal (and (at pack1 cityd)
            (at pack2 cityd)
            (at pack3 cityd) ))
        
)