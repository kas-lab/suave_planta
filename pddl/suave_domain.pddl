(define (domain suave)
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
    owl-number
  )

  (:predicates
    (pipeline_found ?p - pipeline)
    (pipeline_inspected ?p - pipeline)

    (robot_started ?r - robot)
  )

  (:action start_robot
    :parameters (?r - robot)
    :precondition (and
    )
    :effect (and
      (robot_started ?r)
    )
  )

  (:action reconfigure
    :parameters (?f ?fd_initial ?fd_goal)
    :precondition (and
      (not (= ?fd_initial ?fd_goal))

      (Function ?f)
      (FunctionDesign ?fd_initial)
      (functionGrounding ?f ?fd_initial)

      (inferred-SolvesF ?fd_goal ?f)
      (FunctionDesign ?fd_goal)
      (not (inferred-Fd_realisability ?fd_goal false_boolean))

      (or (= ?fd_goal fd_unground)
        (not
          (exists (?fd)
            (and
              (inferred-SolvesF ?fd ?f)
              (not (inferred-Fd_realisability ?fd false_boolean))
              (inferred-Fd_better_utility  ?fd ?fd_goal)
            )
          )
        )
      )
    )
    :effect (and
      (not (functionGrounding ?f ?fd_initial))
      (functionGrounding ?f ?fd_goal)
    )
  )

  (:action reconfigure
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
      (or (= ?fd_goal fd_unground)
        (not
          (exists (?fd)
            (and
              (inferred-SolvesF ?fd ?f)
              (not (inferred-Fd_realisability ?fd false_boolean))
              (inferred-Fd_better_utility  ?fd ?fd_goal)
            )
          )
        )
      )
    )
    :effect (and
      (functionGrounding ?f ?fd_goal)
    )
  )

  (:action search_pipeline
    :parameters (?p - pipeline ?r - robot)
    :precondition (and
      (exists (?a ?f1 ?f2 ?fd1 ?fd2)
        (and
          (inferred-Action ?a)
          (= ?a a_search_pipeline)
          (not (= ?f1 ?f2))
          (inferred-requiresF ?a ?f1)
          (inferred-requiresF ?a ?f2)
          (inferred-F_active ?f1 true_boolean)
          (inferred-F_active ?f2 true_boolean)
        )
      )
      (not (inferred-F_active f_follow_pipeline true_boolean))
      (robot_started ?r)
    )
    :effect (and
      (pipeline_found ?p)
    )
  )

  (:action inspect_pipeline
    :parameters (?p - pipeline ?r - robot)
    :precondition (and
      (exists (?a ?f1 ?f2 ?fd1 ?fd2)
        (and
          (Action ?a)
          (= ?a a_inspect_pipeline)
          (not (= ?f1 ?f2))
          (inferred-requiresF ?a ?f1)
          (inferred-requiresF ?a ?f2)
          (inferred-F_active ?f1 true_boolean)
          (inferred-F_active ?f2 true_boolean)
        )
      )
      (not (inferred-F_active f_generate_search_path true_boolean))
      (robot_started ?r)
      (pipeline_found ?p)
    )
    :effect (and
      (pipeline_inspected ?p)
    )
  )
)
