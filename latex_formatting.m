% Copyright Maciej Lacki 2019
% All rights reserved 

function latex_code = latex_formatting(data)
    make_matrix = true;
    replace_bracets = true;
    upright_text = true;
    remove_space = true;

    latex_code = latex(data);
    if make_matrix
        matches = strfind(latex_code,"\left(\begin{array}"); %20
        % this replaces an arbitrary number of {ccc} with arebitrary length
        for cc = 1:length(matches)
            dd = 1;
            while true
                if latex_code(matches(cc)+19+dd) == 'c'
                    latex_code(matches(cc)+19+dd)
                else
                    break
                end
                dd = dd + 1;
            end
            c_array = "{";
            for dd = 1:dd-1
                c_array = c_array + "c";
            end
            c_array = c_array + "}";
            latex_code = strrep(latex_code,"\left(\begin{array}"+c_array,"\begin{bmatrix}");

        end
        latex_code = strrep(latex_code,"\end{array}\right)","\end{bmatrix}");
    end
    if replace_bracets
        latex_code = strrep(latex_code,"\left","");
        latex_code = strrep(latex_code,"\right","");
    end
    if upright_text
        latex_code = strrep(latex_code,"\mathrm","");
    end
    if remove_space
        latex_code = strrep(latex_code,"\,","");
%         latex_code = strrep(latex_code,"","");
    end
end