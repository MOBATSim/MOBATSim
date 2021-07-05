run('getTestDataColi_noColi.m')

if ~contains(pwd,'Safety Evaluation Report')
    cd('.\src\Misc\Safety Evaluation Report')
end

run('Simplereport.mlx');