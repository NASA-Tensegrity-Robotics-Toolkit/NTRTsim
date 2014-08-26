link_libraries(tgcreator core Adapters)

add_executable(Appduct
    duct/Appduct.cpp
    duct/DuCTTModel.cpp
    duct/tgWorld.cpp
    duct/tgWorldBulletPhysicsImpl.cpp
)

add_executable(AppDuCTT
    DuCTT/AppDuCTT.cpp
    DuCTT/DuCTTModel.cpp
)

add_executable(AppSpine
    Spine/SpineTestModel.cpp
    Spine/AppSpine.cpp
    Spine/SpineRestLengthController.cpp
) 
