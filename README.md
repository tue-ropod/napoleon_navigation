Napoleon Navigation
===================

This package contains an alternative navigation approach as developed within the thesis of Melvin de Wildt: [napoleon_driving
](https://github.com/tue-ropod/ros-structured-nav/tree/feature/napoleon_driving/napoleon_driving)


#### Parameters
Most parameters are specified in [config/NapoleonNavigation.cfg](config/NapoleonNavigation.cfg) so that they are dynamically reconfigurable. To change the default values, only change the corresponding yaml files with the default values for navigation with only ropod ([config/napoleon\_navigation\_ropod.yaml](config/napoelon_navigation_ropod.yaml)), and navigation with ropod and a load ([config/napoleon\_navigation\_load.yaml](config/napoelon_navigation_load.yaml)).

During runtime, the parameters are changed by the [scripts/config\_switcher](scripts/config_switcher) node, which listens to the `/route_navigation/set_load_attached` topic.
