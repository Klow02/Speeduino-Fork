CAN message table tests
=======================
The table logic (src/controllers/canTable/canTable.* and canTableGlue.*) never touches hardware, so
these tests also run on a normal PC without PlatformIO. Needs the Unity test library (unity.c/unity.h):

  g++ -std=c++14 -I speeduino -I speeduino/src/controllers/canTable -I <unity>/src \
      speeduino/src/controllers/canTable/canTable.cpp speeduino/src/controllers/canTable/canTableGlue.cpp \
      test/test_cantable/test_cantable.cpp test/test_cantable/test_glue.cpp \
      <your main() that calls the testCanTable* functions between UNITY_BEGIN() and UNITY_END()> \
      <unity>/src/unity.c -o run_cantable && ./run_cantable
