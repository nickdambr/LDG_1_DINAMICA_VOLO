function factored_expr = expand_roots_into_factors(r)
%EXPAND_ROOTS_INTO_FACTORS Converts a list of roots into a symbolic expression
% with real-coefficient linear and quadratic factors.
%
% Usage:
%   factored_expr = expand_roots_into_factors(r)
%
% Input:
%   r - vector of roots (numeric, possibly complex)
%
% Output:
%   factored_expr - symbolic product of factors (s - root) or expanded quadratics

    syms s
    tol = 1e-8;              % Tolerance for detecting complex conjugates
    used = false(size(r));   % Track used roots
    factors = sym([]);       % Store symbolic factors

    for i = 1:length(r)
        if used(i)
            continue
        end

        ri = r(i);

        if isreal(ri)
            % Real root → linear factor
            factors(end+1) = (s - ri);
            used(i) = true;
        else
            % Look for complex conjugate
            conj_r = conj(ri);
            idx = find(abs(r - conj_r) < tol & ~used);

            if ~isempty(idx)
                % Conjugate found → expand to real quadratic
                a = real(ri);
                b = imag(ri);
                quad = s^2 - 2*a*s + a^2 + b^2;
                factors(end+1) = quad;
                used([i, idx(1)]) = true;
            else
                % No conjugate match → keep as complex linear factor
                factors(end+1) = (s - ri);
                used(i) = true;
            end
        end
    end

    % Multiply all factors into a single expression
    factored_expr = prod(factors);
end
