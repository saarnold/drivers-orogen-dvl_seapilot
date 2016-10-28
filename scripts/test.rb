require 'orocos'
include Orocos

Orocos.initialize

Orocos.run 'dvl_seapilot::Task' => 'dvl' do
    Orocos.conf.load_dir('./')
    dvl = Orocos.name_service.get 'dvl'

    Orocos.conf.apply(dvl, ['default'], :override => true)

    dvl.configure
    dvl.start

    Orocos.watch(dvl)
end

