function Err = RelErr(Truth, Estimation)
    Err = norm(Truth-Estimation,'fro')/norm(Truth,'fro')*100;
end