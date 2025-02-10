function stability_analysis(A)
    % Calculate the eigenvalues of the Jacobian matrix A
    eigenvalues = eig(A);
    disp('Eigenvalues:');
    disp(eigenvalues);

    % Analyze stability based on eigenvalues
    % If all eigenvalues have negative real parts, the system is stable
    stable = all(real(eigenvalues) < 0);
    if stable
        disp('The system is stable.');
    else
        disp('The system is unstable.');
    end
end