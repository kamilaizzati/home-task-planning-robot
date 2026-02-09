(define (domain home-service)
  (:requirements :strips :typing)

  (:types
    object
    location
  )

  (:predicates
    (gripper_free)
    (holding ?o - object)
    (object_at ?o - object ?l - location)
    (object_known ?o - object)
    (object_visible ?o - object)
    (path_found ?from - location ?to - location)
    (robot_at ?l - location)
  )

  (:action pindah
    :parameters (?from - location ?to - location)
    :precondition (and
      (robot_at ?from)
      (path_found ?from ?to)
    )
    :effect (and
      (not (robot_at ?from))
      (robot_at ?to)
    )
  )

  (:action ambil
    :parameters (?o - object ?l - location)
    :precondition (and
      (robot_at ?l)
      (object_at ?o ?l)
      (object_known ?o)
      (gripper_free)
    )
    :effect (and
      (holding ?o)
      (not (object_at ?o ?l))
      (not (gripper_free))
    )
  )

  (:action letak
    :parameters (?o - object ?l - location)
    :precondition (and
      (robot_at ?l)
      (holding ?o)
    )
    :effect (and
      (object_at ?o ?l)
      (gripper_free)
      (not (holding ?o))
    )
  )

  (:action cari
    :parameters (?o - object ?l - location)
    :precondition (and
      (robot_at ?l)
      (object_at ?o ?l)
      (object_known ?o)
    )
    :effect (and
      (object_visible ?o))
  )
)

