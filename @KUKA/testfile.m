            modelKUKA = KUKA;
            modelKUKA.model.base = transl([0 , 0, 0]);
            modelKUKA.model.animate(qrKUKA);
            drawnow();