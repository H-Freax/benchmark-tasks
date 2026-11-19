# Native setup overview

`setup-two-views.png` combines two native1920×1080 RGB camera captures from the same independently reset simulation. The only added labels are the neutral “Setup view1” and “Setup view2” headers. Both native views show the full screen height and base region, the two arms and pedestals, native camera context and target board. No team name, institution, logo, username or local file path is visible.

The documentation cameras were outside the three policy observations. After the normal settled reset, only camera transforms and render-only updates were used. Native task-body and robot transforms, velocities, joints and physics counters were identical before and after those renders. No body pose was rewritten and no reference trajectory was replayed.

The independent reset was compared with a recorded reference reset at physics step480. Task-body position deltas were zero; the maximum native velocity-vector difference was approximately2.63×10⁻⁵, so complete reset-state equality is not claimed. The image depicts the shared geometry/controller/initial-state setup, rather than a full-score terminal state or an image taken from the final model/reference run. Changing a scorer implementation does not make it a new matched-run screenshot.

Several feet and panel surfaces are occluded by other native objects. All six feet cannot be individually verified from these views; use the six-foot floor template, assembly dimensions and actual state evidence. The target dot pattern is visible. The simulator contains no doorway frame or doorway sight markers; their prescribed positions are coordinates for a physical floor template, not marks visible in the screenshot.

The complete full-score reference video has been exported and independently checked (825 frames, 824 controls). The complete Dream-1 video has also been exported and independently checked (1,201 frames,1,200 controls,score0.5). Both videos preserve all three policy camera fields of view at20fps with no cuts; task.json records the uploaded media ID. This setup image alone does not establish stability, support, full score, physical build accuracy or submission readiness.
