Project(tgDataLoggerRedesign)

link_libraries(sensors
                tgcreator
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport
)

# Note, all the classes for this redesign have
# modified names to distinguish from the originals,
# like tgModel_tgDLR.

add_executable( AppT6Model_tgDLR
            T6Model_tgDLR.cpp
	    tgDataObserver_tgDLR.cpp
	    tgDataLoggerRodBasic.cpp
	    tgDataLoggerRodFullState.cpp
	    tgDataLoggerLinearStringBasic.cpp
            T6RestLengthController_tgDLR.cpp
	    AppT6Model_tgDLR.cpp
)


