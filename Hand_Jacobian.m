function [ Jhand ] = Hand_Jacobian( J1, J2, J3 ,Jr1, Jr2, Jc2, Jr3, Jc3 )

Jhand = [               J1*Jr1                  zeros(size(J1*Jr1,1),size(J2*Jr2,2))  zeros(size(J1*Jr1,1),size(J3*Jr3,2))  zeros(size(J1*Jr1,1),size(J3*Jr3,2)) ;...
          zeros(size(J2*Jr2,1),size(J1*Jr1,2))                 J2*Jr2                 zeros(size(J2*Jr2,1),size(J3*Jr3,2))                  J2*Jc2               ;...
          zeros(size(J3*Jr3,1),size(J1*Jr1,2))  zeros(size(J3*Jr3,1),size(J2*Jr2,2))               J3*Jr3                                   J3*Jc3               ];

end

