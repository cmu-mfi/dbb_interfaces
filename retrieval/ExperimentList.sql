/****** Object:  StoredProcedure [dbo].[ExperimentList]    Script Date: 11/7/2024 3:31:35 PM ******/
SET ANSI_NULLS ON
GO
SET QUOTED_IDENTIFIER ON
GO
-- =============================================
-- Author: SSR
-- Create date: <Create Date,,>
-- Description:	Gets the Id, Title, Start and End time from Experiment table
-- =============================================
CREATE PROCEDURE [dbo].[ExperimentList]
    @projectID INT
AS
BEGIN
    -- SET NOCOUNT ON added to prevent extra result sets from
    -- interfering with SELECT statements.
    SET NOCOUNT ON;

    -- Select statements for procedure here
    SELECT ExperimentID, Title, StartTime, EndTime
    FROM dbo.Experiments with (nolock)
    WHERE ProjectID = @projectID
	order by ProjectID desc
	
END
