^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package clearpath_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.5 (2025-03-18)
------------------
* Simplify linear driving test, reduce mobility test logging (`#1 <https://github.com/clearpathrobotics/clearpath_tests/issues/1>`_)
* Contributors: Chris Iverach-Brereton

0.2.4 (2025-03-17)
------------------
* Reduce the minimum duration for a rotation. Log possible false-positives during the rotation test. Print the calculated duration error for the rotation & drive tests
* Log the version of clearpath_tests in the report
* Increase the length of expected lynx messages to 5, cast the length to an integer before comparing it
* Contributors: Chris Iverach-Brereton

0.2.3 (2025-03-14)
------------------
* Invert the angle of the lateral test
* Add a mutex to prevent issues with reading & writing the current & previous orientations asynchronously; this sometimes causes false positives or false negatives during the test
* Don't fail if we get controller_manager rate errors
* Add newline between average motor currents in report
* Increase the allowed margin of error on the IMU test to 20% (from 10%)
* Add an extra confirmation that the lights are in a controllable state before starting the test
* Contributors: Chris Iverach-Brereton

0.2.2 (2025-03-10)
------------------
* Add missing message dependencies
* Contributors: Chris Iverach-Brereton

0.2.1 (2025-03-07)
------------------
* Fix simple_term_menu_vendor dependency
* Contributors: Chris Iverach-Brereton

0.2.0 (2025-03-07)
------------------
* Initial release
* Contributors: Chris Iverach-Brereton, Tony Baltovski
