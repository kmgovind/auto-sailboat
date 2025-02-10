function stability_analysis(A)
    % Calculate the eigenvalues of the Jacobian matrix A
    eigenvalues = eig(A);
    disp('Eigenvalues:');
    disp(eigenvalues);

    % Analyze stability based on eigenvalues
    real_parts = real(eigenvalues);
    if all(real_parts < 0)
        disp('The system is asymptotically stable.');
    elseif all(real_parts <= 0) && any(real_parts == 0)
        disp('The system is marginally stable.');
    else
        disp('The system is unstable.');
    end
end