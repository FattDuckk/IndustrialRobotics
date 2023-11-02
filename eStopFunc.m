function eStopFunc(UR3,KUKA,UR3q0)
            global reset
            popUp=uifigure;
            popUp.Position(3:4) = [350 200];

            %Message, Title
            selection = uiconfirm(popUp,"Warning: Emergency Stop","Program Paused","Options",["Resume", "Reset"],"DefaultOption", 1,"Icon","error");
            switch selection
                case "Resume"
                    eStop=false;
                    close(popUp)
                case "Reset"
                    eStop=false;
                    reset=true;
                    resetBots(UR3,KUKA,UR3q0)
                    close(popUp)
            end
end