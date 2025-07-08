(define (domain suave_extended)
  (:requirements
    :strips
    :typing
    :adl
    :negative-preconditions
    :derived-predicates
    :existential-preconditions
    :disjunctive-preconditions
  )

  (:types
    pipeline
    robot
  )

  (:constants
    bluerov - robot
    0.25_decimal 1.0_decimal - numerical-object 
  )
  
  (:predicates
    (pipeline_found ?p - pipeline)
    (pipeline_inspected ?p - pipeline)

    (robot_started ?r - robot)
    (robot_not_started ?r - robot)

    (inferred-battery_charged ?r - robot)
  )

  (:derived (inferred-battery_charged ?r - robot)
    (and
	    (= ?r bluerov)
      (exists (?mqa ?mqav) 
        (and
          (= ?mqa obs_battery_level)
          (inferred-Qa_has_value ?mqa ?mqav)
          (inferred-IsQAtype ?mqa battery_level)
          (lessThan 0.25_decimal ?mqav)
        )
      )
    )
  )

  (:action start_robot
    :parameters (?r - robot)
    :precondition (and
      (robot_not_started ?r)
    )
    :effect (and
      (not (robot_not_started ?r))
      (robot_started ?r)
    )
  )

  (:action reconfigure1
    :parameters (?f ?fd_goal)
    :precondition (and
      (Function ?f)
      (inferred-SolvesF ?fd_goal ?f)
      (FunctionDesign ?fd_goal)
      (not (inferred-Fd_realisability ?fd_goal false_boolean))
      (not
        (exists (?fd)
          (and
            (inferred-SolvesF ?fd ?f)
            (FunctionDesign ?fd)
            (functionGrounding ?f ?fd)
          )
        )
      )
    )
    :effect (and
      (functionGrounding ?f ?fd_goal)
    )
  )

  (:action reconfigure2
    :parameters (?f ?fd_initial ?fd_goal)
    :precondition (and
      (not (= ?fd_initial ?fd_goal))

      (Function ?f)
      (FunctionDesign ?fd_initial)
      (functionGrounding ?f ?fd_initial)

      (inferred-SolvesF ?fd_goal ?f)
      (FunctionDesign ?fd_goal)
      (not (inferred-Fd_realisability ?fd_goal false_boolean))
    )
    :effect (and
      (not (functionGrounding ?f ?fd_initial))
      (functionGrounding ?f ?fd_goal)
    )
  )

  (:action search_pipeline
    :parameters (?p - pipeline ?r - robot)
    :precondition (and
      (robot_started ?r)
      (inferred-battery_charged ?r)
      (exists (?a ?f1 ?f2 ?fd1 ?fd2)
        (and
          (inferred-Action ?a)
          (= ?a a_search_pipeline)
          (not (= ?f1 ?f2))
          (inferred-requiresF ?a ?f1)
          (inferred-requiresF ?a ?f2)
          (inferred-F_active ?f1 true_boolean)
          (inferred-F_active ?f2 true_boolean)
          (inferred-FunctionGrounding ?f1 ?fd1)
          (inferred-FunctionGrounding ?f2 ?fd2)
          (not (inferred-Fd_realisability ?fd1 false_boolean))
          (not (inferred-Fd_realisability ?fd2 false_boolean))
          (not
            (exists (?fd1_b)
              (and
                (not (= ?fd1 ?fd1_b))
                (inferred-SolvesF ?fd1_b ?f1)
                (not (inferred-Fd_realisability ?fd1_b false_boolean))
                (inferred-FdBetterUtility  ?fd1_b ?fd1)
              )
            )
          )
          (not
            (exists (?fd2_b)
              (and
                (not (= ?fd2 ?fd2_b))
                (inferred-SolvesF ?fd2_b ?f2)
                (not (inferred-Fd_realisability ?fd2_b false_boolean))
                (inferred-FdBetterUtility  ?fd2_b ?fd2)
              )
            )
          )
        )
      )
      (not (inferred-F_active f_follow_pipeline true_boolean))
      (not (inferred-F_active generate_recharge_path true_boolean))
    )
    :effect (and
      (pipeline_found ?p)
    )
  )

  (:action inspect_pipeline
    :parameters (?p - pipeline ?r - robot)
    :precondition (and
      (robot_started ?r)
      (pipeline_found ?p)
      (inferred-battery_charged ?r)
      (exists (?a ?f1 ?f2 ?fd1 ?fd2)
        (and
          (inferred-Action ?a)
          (= ?a a_inspect_pipeline)
          (not (= ?f1 ?f2))
          (inferred-requiresF ?a ?f1)
          (inferred-requiresF ?a ?f2)
          (inferred-F_active ?f1 true_boolean)
          (inferred-F_active ?f2 true_boolean)
          (inferred-FunctionGrounding ?f1 ?fd1)
          (inferred-FunctionGrounding ?f2 ?fd2)
          (not (inferred-Fd_realisability ?fd1 false_boolean))
          (not (inferred-Fd_realisability ?fd2 false_boolean))
          (not
            (exists (?fd1_b)
              (and
                (not (= ?fd1 ?fd1_b))
                (inferred-SolvesF ?fd1_b ?f1)
                (not (inferred-Fd_realisability ?fd1_b false_boolean))
                (inferred-FdBetterUtility  ?fd1_b ?fd1)
              )
            )
          )
          (not
            (exists (?fd2_b)
              (and
                (not (= ?fd2 ?fd2_b))
                (inferred-SolvesF ?fd2_b ?f2)
                (not (inferred-Fd_realisability ?fd2_b false_boolean))
                (inferred-FdBetterUtility  ?fd2_b ?fd2)
              )
            )
          )
        )
      )
      (not (inferred-F_active f_generate_search_path true_boolean))
      (not (inferred-F_active generate_recharge_path true_boolean))
    )
    :effect (and
      (pipeline_inspected ?p)
    )
  )

  (:action recharge_battery
    :parameters (?r - robot)
    :precondition (and
      (robot_started ?r)
      (exists (?a ?f1 ?f2 ?fd1 ?fd2)
        (and
          (inferred-Action ?a)
          (= ?a a_recharge_battery)
          (not (= ?f1 ?f2))
          (inferred-requiresF ?a ?f1)
          (inferred-requiresF ?a ?f2)
          (inferred-F_active ?f1 true_boolean)
          (inferred-F_active ?f2 true_boolean)
          (inferred-FunctionGrounding ?f1 ?fd1)
          (inferred-FunctionGrounding ?f2 ?fd2)
          (not (inferred-Fd_realisability ?fd1 false_boolean))
          (not (inferred-Fd_realisability ?fd2 false_boolean))
        )
      )
      (not (inferred-F_active f_generate_search_path true_boolean))
      (not (inferred-F_active f_follow_pipeline true_boolean))
    )
    :effect (and
      (qa_has_value obs_battery_level 1.0_decimal)
    )
  )

)
