(define (problem example-auv)
  (:domain suave_extended)

  (:objects
    pipeline - pipeline
  )

  (:init
    ;(qa_has_value obs_water_visibility 2.5_decimal)
    (qa_has_value obs_battery_level 1.0_decimal)
    (robot_not_started bluerov)
  )

  (:goal (and
      (robot_started bluerov)
      (pipeline_found pipeline)
      (pipeline_inspected pipeline)
    )
  )
)
