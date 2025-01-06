(define (problem example-auv)
  (:domain suave)

  (:objects
    pipeline - pipeline
    bluerov - robot
    1.0_decimal 1.5_decimal 3.5_decimal - owl-number
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
