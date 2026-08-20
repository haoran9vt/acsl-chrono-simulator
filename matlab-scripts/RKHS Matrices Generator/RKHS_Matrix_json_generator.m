%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2026 Haoran Wang, Andrea L'Afflitto. All rights reserved.                                                        
%                                                                             
% Redistribution and use in source and binary forms, with or without          
% modification, are permitted provided that the following conditions 
% are met: 
%                                                                             
% 1. Redistributions of source code must retain the above copyright notice,   
%    this list of conditions and the following disclaimer.                    
%                                                                             
% 2. Redistributions in binary form must reproduce the above copyright        
%    notice, this list of conditions and the following disclaimer in the      
%    documentation and/or other materials provided with the distribution.     
%                                                                             
% 3. Neither the name of the copyright holder nor the names of its            
%    contributors may be used to endorse or promote products derived from     
%    this software without specific prior written permission.                 
%                                                                             
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
% PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER 
% OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:       adaptive_rotational_differentiator_parameter_generator
% Author:     Haoran Wang                                             
% Date:       August 19, 2026
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: Main function to compute the params for the adaptive 
%              differentiator for the simulator and flight.
% 
% Github: https://github.com/andrealaffly/acsl-physics-sim.git
%
% Note: AI was used to generate code for 'JSON Export Script' - Haoran.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% House Keeping
clear all; close all; clc

%% Define Parameters
rangeX=[-5,5];
rangeY=[-5,5];
rangeZ=[-5,5];
N=3;
m=3;
l=2;
knlfun = @(x,y) exp(-1/(2*l^2)*norm(x-y,2).^2)*eye(3);

%% Calculating Key Matrices
xcentergrid=linspace(rangeX(1),rangeX(2),N);                % x coordinates of centers
ycentergrid=linspace(rangeY(1),rangeY(2),N);                % y coordinates of centers
zcentergrid=linspace(rangeX(1),rangeX(2),N);                % y coordinates of centers

[Xcenter,Ycenter,Zcenter]=meshgrid(xcentergrid,ycentergrid,zcentergrid);        % creating meshgrid

basiscenter=[Xcenter(:),Ycenter(:),Zcenter(:)]';                       % Assigning basiscenters

hN=sqrt(((rangeX(2)-rangeX(1))/(N-1))^2+...
    ((rangeY(2)-rangeY(1))/(N-1))^2+...
    ((rangeZ(2)-rangeZ(1))/(N-1))^2)/2;   % Calculate fill distance for regular grid

% Calculating the Grammian (big K) matrix
k_xi_xi=zeros(length(basiscenter)*m);                         % Initialize the Grammian matrix

for i=1:length(basiscenter)                                 % Double for loop to assign components as defined
    for j=1:length(basiscenter)
        k_xi_xi(1+m*(i-1):m+m*(i-1),1+m*(j-1):m+m*(j-1))=knlfun(basiscenter(:,i),basiscenter(:,j));
    end
end
cond_K=cond(k_xi_xi);                                       % Condition number of the Grammian (big K) matrix
inv_k_xi_xi=inv(k_xi_xi);                                   % inverse of the Grammian (big K) matrix

%% Export the desired matrices to a json file
fid=fopen('RKHS_Matrices.json', 'w');
for i=1:3
    center_json=jsonencode(basiscenter(i,:));
    fprintf(fid, '%s,\n', center_json);
end
fprintf(fid, '\n');
for i=1:length(inv_k_xi_xi)
    inv_k_xi_xi_json=jsonencode(round(inv_k_xi_xi(i,:),4));
    fprintf(fid, '%s,\n', inv_k_xi_xi_json);
end
fclose(fid);