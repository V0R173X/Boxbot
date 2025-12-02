function send_to_joint(arg,q)
switch arg
    case 1
        set_param('model/qdes/base', 'Value', num2str(q));
    case 2
        set_param('model/qdes/stepper', 'Value', num2str(q));
    case 3
        set_param('model/qdes/arm', 'Value', num2str(q));
    case 4
        set_param('model/qdes/gripper','Value',num2str(q))
            
end
    

end

