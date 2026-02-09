(define (problem home-service-prob)
  (:domain home-service)

  (:objects
    gelas piring botol sendok apel baju - object
    dapur meja rak ruangtamu lemari kamar jendela pintu koridor - location
  )

  (:init
    (robot_at dapur)
    (gripper_free)

    (object_at gelas meja)
    (object_at apel meja)
    (object_at sendok meja)
    (object_at botol rak)
    (object_at piring lemari)
    (object_at baju ruangtamu)

    (object_known gelas)
    (object_known sendok)
    (object_known apel)
    (object_known botol)
    (object_known piring)
    (object_known baju)

    ; gripper kosong
    (gripper_free)

    (path_found dapur meja)
    (path_found dapur rak)
    (path_found dapur ruangtamu)
    (path_found dapur lemari)
    (path_found dapur kamar)
    (path_found dapur pintu)

    (path_found meja dapur)
    (path_found rak dapur)
    (path_found ruangtamu dapur)
    (path_found lemari dapur)
  )

  (:goal
    (and (robot_at rak))
  )
)

