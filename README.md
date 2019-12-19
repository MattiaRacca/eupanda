# EUPanda: a End-User Programming framework for the FRANKA Panda

**EUPanda** is a End-User Programming framework for the FRANKA Panda Robot Arm that tries to replicate the proprietary
[Desk environment] in a open source ROS-based fashion.

The framework allows to create from kinestethic teaching robot programs made of 5 primitives (move to, move to contact,
move fingers, apply force with fingers and an user synchronization primitive). Programs can be executed step-by-step or
fully; Similarly, primitives can be reverted.

This is still Work In Progress! Contact me (Mattia Racca) for more information!

For the Active Learning code behind our HRI'20 paper **"Interactive Tuning of Robot Program Parameters via Expected
Divergence Maximization", Mattia Racca, Ville Kyrki, and Maya Cakmak"**, please check the [tuning_al_gui] package.

[tuning_al_gui]: https://github.com/MattiaRacca/tuning_al_gui
[Desk environment]: https://www.franka.de/capability

#### License
Everything authored by me is released under the GNU GPL 3.0 license. Some files are authored by others, and are included
as part of my own setup; they still belong to the original authors.
