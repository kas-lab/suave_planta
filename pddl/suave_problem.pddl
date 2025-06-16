(define (problem example-auv)
  (:domain suave)

  (:objects
    pipeline - pipeline
    bluerov - robot
  )

  (:init
    ;(qa_has_value obs_water_visibility 2.5_decimal)
    (robot_not_started bluerov)
  )

  (:goal (and
      (robot_started bluerov)
      (pipeline_found pipeline)
      (pipeline_inspected pipeline)
    )
  )
)
