(define (problem example-auv)
  (:domain suave)

  (:objects
    pipeline - pipeline
    bluerov - robot
  )

  (:init
    ;(hasValue obs_water_visibility 2.5_decimal)
  )

  (:goal (and
      (robot_started bluerov)
      (pipeline_found pipeline)
      (pipeline_inspected pipeline)
    )
  )
)
